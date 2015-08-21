// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>

#include <string>
#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "planar5_dyn_model.h"
#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/utilities.h"
#include "planer_utils/random_uniform.h"
#include "planer_utils/reachability_map.h"
#include "planer_utils/rrt_star.h"
#include "planer_utils/simulator.h"

class PathsGenerator {
protected:
    boost::shared_ptr<ReachabilityMap > r_map_;
    boost::shared_ptr<ReachabilityMap > r_map_tmp_;
    boost::shared_ptr<RRTStar > rrt_;
    int ndim_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;
    Eigen::VectorXd xs_, xe_;
public:
    PathsGenerator(int ndim, const Eigen::VectorXd &lower_bound, const Eigen::VectorXd &upper_bound, const boost::shared_ptr<self_collision::CollisionModel> &col_model) :
        ndim_(ndim),
        lower_bound_(lower_bound),
        upper_bound_(upper_bound)
    {
        r_map_.reset( new ReachabilityMap(0.05, ndim_) );
        r_map_->generate(lower_bound, upper_bound);

        r_map_tmp_.reset( new ReachabilityMap(0.05, ndim_) );
        r_map_tmp_->generate(lower_bound, upper_bound);

        rrt_.reset( new RRTStar(ndim_, boost::bind(&PathsGenerator::checkCollision, this, _1, col_model),
                    boost::bind(&PathsGenerator::costLine, this, _1, _2), boost::bind(&PathsGenerator::sampleSpace, this, _1), 0.05, 0.2, 0.4 ) );
    }

    void reset(const Eigen::VectorXd &xs, const Eigen::VectorXd &xe) {
        xs_ = xs;
        xe_ = xe;
        r_map_->clear();
    }

    void sampleSpace(Eigen::VectorXd &sample) const {
        for (int dim_idx = 0; dim_idx < ndim_; dim_idx++) {
            sample(dim_idx) = randomUniform(lower_bound_(dim_idx), upper_bound_(dim_idx));
        }
    }

    bool checkCollision(const Eigen::VectorXd &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.1, KDL::Frame(KDL::Vector(x[0], x[1], 0)));
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("base")), T_B_L2);
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);

        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("base")), T_B_L2) || self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const {
        Eigen::VectorXd v = x2-x1;
        double norm = v.norm();
        double step_len = 0.05;

        if (r_map_->getMaxValue() == 0.0) {
            return 1.0 * norm;
        }
        return ((r_map_->getMaxValue() * r_map_->getValue((x1+x2)/2.0)) + 1.0) * norm;
    }

    bool getPath(std::list<Eigen::VectorXd > &path) {
        for (int i = 0; i < 10; i++) {
            path.clear();
            rrt_->plan(xs_, xe_, 0.05, path);
            if (path.size() > 0) {
                break;
            }
        }

        if (path.size() == 0) {
            return false;
        }

        r_map_tmp_->clear();
        for (double f = 0.0; f < 1.0; f += 0.001) {
            Eigen::VectorXd pt(2);
            getPointOnPath(path, f, pt);
            r_map_tmp_->setValue(pt, 1);
        }

        r_map_tmp_->grow();
        r_map_tmp_->grow();
        r_map_->addMap(r_map_tmp_);

        return true;
    }

};

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

    std::list<Eigen::VectorXd > penalty_points_;

public:
    TestDynamicModel() :
        nh_(),
        PI(3.141592653589793),
        markers_pub_(nh_)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }

    ~TestDynamicModel() {
    }

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const boost::shared_ptr<ReachabilityMap > &r_map) const {
        return (x1-x2).norm() * (2.0 - r_map->getValue(x1) - r_map->getValue(x2));
    }

    double costLine2(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const boost::shared_ptr<ReachabilityMap > &r_map) const {
        Eigen::VectorXd v = x2-x1;
        double norm = v.norm();
        double step_len = 0.05;

        if (r_map->getMaxValue() == 0.0) {
            return 1.0 * norm;
        }
        return ((r_map->getMaxValue() * r_map->getValue((x1+x2)/2.0)) + 1.0) * norm;
    }

    bool checkCollision(const Eigen::VectorXd &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.1, KDL::Frame(KDL::Vector(x[0], x[1], 0)));
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("base")), T_B_L2);
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);

        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("base")), T_B_L2) || self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    void spin() {

        // initialize random seed
        srand(time(NULL));

        // dynamics model
        boost::shared_ptr<DynamicModel > dyn_model( new DynModelPlanar5() );

        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

        //
        // collision model
        //
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);
	    col_model->parseSRDF(robot_semantic_description_str);
        col_model->generateCollisionPairs();

        // external collision objects - part of virtual link connected to the base link
        self_collision::Link::VecPtrCollision col_array;
//        col_array.push_back( self_collision::createCollisionCapsule(0.2, 0.3, KDL::Frame(KDL::Vector(1, 0.5, 0))) );
        col_array.push_back( self_collision::createCollisionCapsule(0.05, 0.3, KDL::Frame(KDL::Rotation::RotX(90.0/180.0*PI), KDL::Vector(1, 0.2, 0))) );
        col_array.push_back( self_collision::createCollisionCapsule(0.05, 0.2, KDL::Frame(KDL::Rotation::RotZ(90.0/180.0*PI)*KDL::Rotation::RotX(90.0/180.0*PI), KDL::Vector(0.9, 0.35, 0))) );
        if (!col_model->addLink("env_link", "base", col_array)) {
            ROS_ERROR("ERROR: could not add external collision objects to the collision model");
            return;
        }
        col_model->generateCollisionPairs();

        //
        // robot state
        //
        std::vector<std::string > joint_names;
        joint_names.push_back("0_joint");
        joint_names.push_back("1_joint");
        joint_names.push_back("2_joint");
        joint_names.push_back("3_joint");
        joint_names.push_back("4_joint");

        int ndof = joint_names.size();

        Eigen::VectorXd saved_q(ndof), saved_dq(ndof), saved_ddq(ndof);
        saved_q.resize( ndof );
        saved_dq.resize( ndof );
        saved_ddq.resize( ndof );

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            saved_q[q_idx] = -0.1;
            saved_dq[q_idx] = 0.0;
            saved_ddq[q_idx] = 0.0;
        }

        std::string effector_name = "effector";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel> kin_model( new KinematicModel(robot_description_str, joint_names) );

        KinematicModel::Jacobian J_r_HAND_6, J_r_HAND;
        J_r_HAND_6.resize(6, ndof);
        J_r_HAND.resize(3, ndof);

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof), limit_range(ndof), max_trq(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {

            if (!col_model->getJointLimits( (*name_it), lower_limit[q_idx], upper_limit[q_idx] )) {
                ROS_ERROR("ERROR: could not find joint with name %s", name_it->c_str() );
                return;
            }
            limit_range[q_idx] = 10.0/180.0*PI;
            max_trq[q_idx] = 10.0;
        }

        ros::Duration(1.0).sleep();

        Eigen::VectorXd lower_bound(2);
        Eigen::VectorXd upper_bound(2);
        lower_bound(0) = -0.5;
        upper_bound(0) = 2.0;
        lower_bound(1) = -1.0;
        upper_bound(1) = 1.0;

        PathsGenerator pg(2, lower_bound, upper_bound, col_model);

        DynamicsSimulatorHandPose sim(ndof, 3, effector_name, col_model, kin_model, dyn_model, joint_names);
        sim.setState(saved_q, saved_dq, saved_ddq);

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        ros::Rate loop_rate(500);
        std::list<Eigen::VectorXd > target_path;

        KDL::Twist diff_target;
        KDL::Frame r_HAND_start;

        while (ros::ok()) {

            Eigen::VectorXd xe(2);
            bool pose_found = false;
            // set new random target pose
            for (int i = 0; i < 100; i++) {
                r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(0,1.8), randomUniform(-1,1), 0));
                xe(0) = r_HAND_target.p.x();
                xe(1) = r_HAND_target.p.y();
                if (!checkCollision(xe, col_model)) {
                    pose_found = true;
                    break;
                }
            }
            if (!pose_found) {
                std::cout << "ERROR: could not find valid pose" << std::endl;
                return;
            }
            publishTransform(br, r_HAND_target, "effector_dest", "base");

            // get the current pose
            sim.getState(saved_q, saved_dq, saved_ddq);

            KDL::Frame T_B_E;
            kin_model->calculateFk(T_B_E, effector_name, saved_q);

            Eigen::VectorXd xs(2);
            xs(0) = T_B_E.p.x();
            xs(1) = T_B_E.p.y();

            pg.reset(xs, xe);
            r_HAND_start = T_B_E;
            diff_target = KDL::diff(r_HAND_start, r_HAND_target, 1.0);

            bool pose_reached  = false;

            for (int path_idx = 0; path_idx < 10; path_idx++) {

                std::cout << "replanning the path..." << std::endl;
                std::list<Eigen::VectorXd > path;
                if (!pg.getPath(path)) {
                    std::cout << "could not plan end effector path" << std::endl;
                    continue;
                }
                target_path = path;
                double sim_loops_per_meter = 2000.0;
                int sim_loops = static_cast<int>( getPathLength(path) * sim_loops_per_meter );

                // visualize the planned graph
                int m_id = 100;
//                m_id = rrt.addTreeMarker(markers_pub_, m_id);
                m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xe(0), xe(1), 0), 0, 0, 1, 1, 0.05, "base");
                for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
                    KDL::Vector pos1((*it1)(0), (*it1)(1), 0), pos2((*it2)(0), (*it2)(1), 0);
                    m_id = markers_pub_.addVectorMarker(m_id, pos1, pos2, 1, 1, 1, 1, 0.02, "base");
                }

                markers_pub_.addEraseMarkers(m_id, 2000);
                markers_pub_.publish();
                ros::spinOnce();
                ros::Duration(0.01).sleep();

                for (int rot_idx = 0; rot_idx < 3; rot_idx++) {
                    if (rot_idx == 1) {
                        double angle = diff_target.rot.Norm();
                        diff_target.rot = (angle - 360.0/180.0*PI) * diff_target.rot / angle;
                        std::cout << "trying other rotation..." << std::endl;
                    }
                    else if (rot_idx == 2) {
                        double angle = diff_target.rot.Norm();
                        diff_target.rot = (angle - 2.0*360.0/180.0*PI) * diff_target.rot / angle;
                        std::cout << "trying other rotation..." << std::endl;
                    }

                    sim.setState(saved_q, saved_dq, saved_ddq);

                    for (int loop_counter = 0; loop_counter < sim_loops; loop_counter++) {
                        Eigen::VectorXd pt(2);
                        double f_path = static_cast<double >(loop_counter)/(sim_loops*2/3);
                        if (f_path > 1.0) {
                            f_path = 1.0;
                        }
                        getPointOnPath(target_path, f_path, pt);
                        markers_pub_.addSinglePointMarker(50, KDL::Vector(pt(0), pt(1), 0), 1, 0, 0, 1, 0.2, "base");

                        Eigen::VectorXd q(ndof), dq(ndof), ddq(ndof);
                        sim.getState(q, dq, ddq);
                        KDL::Frame T_B_E;
                        kin_model->calculateFk(T_B_E, effector_name, q);
                        KDL::Frame r_HAND_current = T_B_E;

                        KDL::Rotation target_rot = KDL::addDelta(r_HAND_start, diff_target, f_path).M;

                        KDL::Frame target_pos(target_rot, KDL::Vector(pt(0), pt(1), 0));
                        KDL::Twist diff = KDL::diff(r_HAND_current, target_pos, 1.0);
                        sim.oneStep(diff);

                        KDL::Twist diff_goal = KDL::diff(r_HAND_current, r_HAND_target, 1.0);

                        if (diff_goal.vel.Norm() < 0.06 && diff.rot.Norm() < 5.0/180.0 * PI) {
            //                std::cout << "Pose reached " << diff.vel.Norm() << " " << diff.rot.Norm() << std::endl;
                            pose_reached  = true;
                            break;
                        }

                        // publish markers and robot state with limited rate
                        ros::Duration time_elapsed = ros::Time::now() - last_time;
                        if (time_elapsed.toSec() > 0.05) {
                            // calculate forward kinematics for all links
                            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                            }
                            publishJointState(joint_state_pub_, q, joint_names);
                            int m_id = 0;
                            m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                            markers_pub_.publish();
                            ros::Time last_time = ros::Time::now();
                        }
                        ros::spinOnce();
//                        loop_rate.sleep();
                    }
                    if (pose_reached) {
                        break;
                    }
                }
                if (pose_reached) {
                    break;
                }
            }


            if (pose_reached) {
                std::cout << "pose reached" << std::endl;
            }
            else {
                std::cout << "could not reach the pose" << std::endl;
            }
            getchar();
        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}

