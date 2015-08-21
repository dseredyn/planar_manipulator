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
#include "rrt.h"

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

    bool checkCollision2(const KDL::Frame &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol = self_collision::createCollisionSphere(0.1, x);
        KDL::Frame T_B_L1;
        KDL::Frame T_B_L2;
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("base")), T_B_L2);
        self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);

        return self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("base")), T_B_L2) || self_collision::checkCollision(pcol, T_B_L1, col_model->getLink(col_model->getLinkIndex("env_link")), T_B_L2);
    }

    void sampleSpace(KDL::Frame &x, const Eigen::VectorXd &lower_bound, const Eigen::VectorXd &upper_bound) {
        x = KDL::Frame( KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(lower_bound(0), upper_bound(0)), randomUniform(lower_bound(1), upper_bound(1)), 0) );
    }

    void generatePossiblePose(KDL::Frame &T_B_E, int ndof, const std::string &effector_name, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const boost::shared_ptr<KinematicModel> &kin_model) {
        while (true) {
            Eigen::VectorXd q(ndof);
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                q(q_idx) = randomUniform(kin_model->getLowerLimit(q_idx), kin_model->getUpperLimit(q_idx));
            }
            std::set<int> excluded_link_idx;
            std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }

            if (!self_collision::checkCollision(col_model, links_fk, excluded_link_idx)) {
                T_B_E = links_fk[col_model->getLinkIndex(effector_name)];
                break;
            }
        }
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

        boost::shared_ptr<DynamicsSimulatorHandPose> sim( new DynamicsSimulatorHandPose(ndof, 3, effector_name, col_model, kin_model, dyn_model, joint_names) );
        sim->setState(saved_q, saved_dq, saved_ddq);

        RRT rrt(ndof, boost::bind(&TestDynamicModel::checkCollision2, this, _1, col_model),
                boost::bind(&TestDynamicModel::sampleSpace, this, _1, lower_bound, upper_bound),
                0.05, 0.5, 0.8, kin_model, effector_name, sim);

        PathsGenerator pg(2, lower_bound, upper_bound, col_model);


        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        ros::Rate loop_rate(500);
        std::list<Eigen::VectorXd > target_path;

        KDL::Twist diff_target;
        KDL::Frame r_HAND_start;

        while (ros::ok()) {

            Eigen::VectorXd xe(2);

            generatePossiblePose(r_HAND_target, ndof, effector_name, col_model, kin_model);
            xe(0) = r_HAND_target.p.x();
            xe(1) = r_HAND_target.p.y();

            publishTransform(br, r_HAND_target, "effector_dest", "base");

            // get the current pose
            sim->getState(saved_q, saved_dq, saved_ddq);

            KDL::Frame T_B_E;
            kin_model->calculateFk(T_B_E, effector_name, saved_q);

            std::list<KDL::Frame > path_x;
            std::list<Eigen::VectorXd > path_q;
            rrt.plan(saved_q, r_HAND_target, 0.05, path_x, path_q, markers_pub_);

            std::cout << "planning ended " << path_x.size() << std::endl;

            // execute the planned path
            sim->setState(saved_q, saved_dq, saved_ddq);

            for (std::list<KDL::Frame >::const_iterator it = path_x.begin(); it != path_x.end(); it++) {
                sim->setTarget( (*it) );
                KDL::Twist diff_target = KDL::diff( T_B_E, (*it), 1.0);

                for (int loop_counter = 0; loop_counter < 3000; loop_counter++) {
                    Eigen::VectorXd q(ndof), dq(ndof), ddq(ndof);
                    sim->getState(q, dq, ddq);

                    KDL::Frame r_HAND_current;
                    kin_model->calculateFk(r_HAND_current, effector_name, q);

                    KDL::Twist goal_diff( KDL::diff(r_HAND_current, (*it), 1.0) );
                    if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                        break;
                    }
                    sim->oneStep();

                    sim->getState(q, dq, ddq);
                    // calculate forward kinematics for all links
                    for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                        kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                    }
                    publishJointState(joint_state_pub_, q, joint_names);//, ign_q, ign_joint_names);
                    int m_id = 0;
                    m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                    markers_pub_.publish();
                    ros::spinOnce();
                    loop_rate.sleep();
                }
            }
//            sim->setState(saved_q, saved_dq, saved_ddq);

            getchar();
            markers_pub_.addEraseMarkers( 0, 2000);
            markers_pub_.publish();
            ros::spinOnce();


        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}

