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
#include "kin_model.h"
#include "marker_publisher.h"
#include "task_col.h"
#include "task_hand.h"
#include "task_jlc.h"
#include "planar_collision.h"
#include "random_uniform.h"
#include "reachability_map.h"
#include "rrt_star.h"

class SphereQ {
public:
    SphereQ(const Eigen::VectorXd &center, double radius) :
        center_(center),
        radius_(radius)
    {
    }

    SphereQ(const SphereQ &s) :
        center_(s.center_),
        radius_(s.radius_)
    {
    }

    Eigen::VectorXd center_;
    double radius_;
};

class Task_QA {
public:
    Task_QA(int ndof, const Eigen::VectorXd &lower_limit, const Eigen::VectorXd &upper_limit) :
        ndof_(ndof),
        lower_limit_(lower_limit),
        upper_limit_(upper_limit)
    {
    }

    ~Task_QA() {
    }

    void compute(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &I, const std::vector<SphereQ> &spheres, Eigen::VectorXd &torque, Eigen::MatrixXd &N) {

            double activation = 0.0;
            double max_force = 1.0;

            for (int q_idx = 0; q_idx < ndof_; q_idx++) {
//                torque(q_idx) = -0.1 * 2.0 * (1.0/(upper_limit_(q_idx) - lower_limit_(q_idx)) / (upper_limit_(q_idx) - lower_limit_(q_idx))) * (q(q_idx) - 1.0);
                torque(q_idx) = -20.0 * (q(q_idx) - 1.0);
            }
            double norm = torque.norm();
            if (norm > max_force) {
                torque = torque / norm * max_force;
            }

    }

protected:
    int ndof_;
    Eigen::VectorXd lower_limit_;
    Eigen::VectorXd upper_limit_;
};

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

    std::list<Eigen::VectorXd > penalty_points_;
    ReachabilityMap r_map_;

public:
    TestDynamicModel() :
        nh_(),
        PI(3.141592653589793),
        markers_pub_(nh_),
        r_map_(0.1, 2)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }

    ~TestDynamicModel() {
    }

    void publishJointState(const Eigen::VectorXd &q, const std::vector<std::string > &joint_names) {
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator it = joint_names.begin(); it != joint_names.end(); it++, q_idx++) {
            js.name.push_back(*it);
            js.position.push_back(q[q_idx]);
        }
        joint_state_pub_.publish(js);
    }

    void publishTransform(const KDL::Frame &T_B_F, const std::string &frame_id) {
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(T_B_F.p.x(), T_B_F.p.y(), T_B_F.p.z()) );
        tf::Quaternion q;
        double qx, qy, qz, qw;
        T_B_F.M.GetQuaternion(q[0], q[1], q[2], q[3]);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", frame_id));
    }

    int publishRobotModelVis(int m_id, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &T) {
        for (self_collision::CollisionModel::VecPtrLink::const_iterator l_it = col_model->getLinks().begin(); l_it != col_model->getLinks().end(); l_it++) {
            KDL::Frame T_B_L = T[(*l_it)->index_];
            for (self_collision::Link::VecPtrCollision::const_iterator it = (*l_it)->collision_array.begin(); it != (*l_it)->collision_array.end(); it++) {
                KDL::Frame T_B_O = T_B_L * (*it)->origin;
                if ((*it)->geometry->type == self_collision::Geometry::CONVEX) {
                    // TODO
                }
                else if ((*it)->geometry->type == self_collision::Geometry::SPHERE) {
                    self_collision::Sphere *sphere = static_cast<self_collision::Sphere* >((*it)->geometry.get());
                    m_id = markers_pub_.addSinglePointMarker(m_id, T_B_O.p, 0, 1, 0, 1, sphere->radius*2, "base");
                }
                else if ((*it)->geometry->type == self_collision::Geometry::CAPSULE) {
                    self_collision::Capsule *capsule = static_cast<self_collision::Capsule* >((*it)->geometry.get());
                    m_id = markers_pub_.addCapsule(m_id, T_B_O, capsule->length, capsule->radius, "base");
                }
            }
        }
        return m_id;
    }

    bool checkCollision(const Eigen::VectorXd &x, const std::vector<KDL::Frame > &links_fk, const boost::shared_ptr<self_collision::CollisionModel> &col_model) {
        // check collision with base and environment only
        std::vector<int > static_links_idx;
        static_links_idx.push_back( col_model->getLinkIndex("base") );
        static_links_idx.push_back( col_model->getLinkIndex("env_link") );

        // create dummy object
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Sphere());
        boost::shared_ptr<self_collision::Sphere > sph = boost::static_pointer_cast<self_collision::Sphere >(pcol->geometry);
        sph->radius = 0.1;
        pcol->origin = KDL::Frame(KDL::Vector(x[0], x[1], 0));
        int base_link_idx = col_model->getLinkIndex("base");
        const KDL::Frame &T_B_L1 = links_fk[base_link_idx];

        for (std::vector<int >::const_iterator li_it = static_links_idx.begin(); li_it != static_links_idx.end(); li_it++) {
            int link2_idx = *li_it;
            const KDL::Frame &T_B_L2 = links_fk[link2_idx];
            for (self_collision::Link::VecPtrCollision::const_iterator col2_it = col_model->getLinkCollisionArray(link2_idx).begin(); col2_it != col_model->getLinkCollisionArray(link2_idx).end(); col2_it++) {
                double dist = 0.0;
                KDL::Vector p1_B, p2_B, n1_B, n2_B;
                if (::checkCollision(pcol, *col2_it, T_B_L1, T_B_L2, 0.0, dist, p1_B, p2_B, n1_B, n2_B)) {
                    return true;
                }
            }
        }

        return false;
    }

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const {
        return (x1-x2).norm() * (2.0 - r_map_.getValue(x1) - r_map_.getValue(x2));
    }

    void sampleSpace(Eigen::VectorXd &sample) const {
        sample(0) = randomUniform(0,1.7);
        sample(1) = randomUniform(-1,1);
    }

    bool checkCollisionQ5(const Eigen::VectorXd &x, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const KinematicModel &kin_model) {
        std::vector<CollisionInfo> link_collisions;
        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
        }
        getCollisionPairs(col_model, links_fk, 0.05, link_collisions);
        if (link_collisions.size() > 0) {
            return true;
        }

        return false;
    }

    double costLineQ5(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const {
        return (x1-x2).norm();
    }

    void sampleSpaceQ5(Eigen::VectorXd &sample, const Eigen::VectorXd &lower_limit, const Eigen::VectorXd &upper_limit) const {
        for (int q_idx = 0; q_idx < 5; q_idx++) {
            sample(q_idx) = randomUniform(lower_limit(q_idx), upper_limit(q_idx));
        }
    }

    void getPointOnPath(const std::list<Eigen::VectorXd > &path, double f, Eigen::VectorXd &x) const {

        if (path.size() == 0) {
            std::cout << "ERROR: getPointOnPath: path size is 0" << std::endl;
            return;
        }
        else if (path.size() == 1 || f < 0.0) {
            x = (*path.begin());
            return;
        }

        if (f > 1.0) {
            x = (*(--path.end()));
            return;
        }


        double length = 0.0;
        for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
            double dist = ((*it1) - (*it2)).norm();
            length += dist;
        }

        double pos = length * f;

        for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
            Eigen::VectorXd v = ((*it2) - (*it1));
            double dist = v.norm();
            if (pos - dist > 0) {
                pos -= dist;
            }
            else {
                x = (*it1) + pos * v / dist;
                return;
            }
        }
        std::cout << "ERROR: getPointOnPath: ??" << std::endl;
    }

    boost::shared_ptr< self_collision::Collision > createCollisionCapsule(double radius, double length, const KDL::Frame &origin) const {
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Capsule());
        boost::shared_ptr<self_collision::Capsule > cap = boost::static_pointer_cast<self_collision::Capsule >(pcol->geometry);
        cap->radius = radius;
        cap->length = length;
        pcol->origin = origin;
        return pcol;
    }

    void spin() {

        // initialize random seed
        srand(time(NULL));

        // dynamics model
        DynModelPlanar5 dyn_model;

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
//        col_array.push_back( createCollisionCapsule(0.2, 0.3, KDL::Frame(KDL::Vector(1, 0.5, 0))) );
        col_array.push_back( createCollisionCapsule(0.05, 0.3, KDL::Frame(KDL::Vector(1, 0.2, 0))) );
        col_array.push_back( createCollisionCapsule(0.05, 0.2, KDL::Frame(KDL::Rotation::RotZ(90.0/180.0*PI), KDL::Vector(0.9, 0.35, 0))) );
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

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = -0.1;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
        }

        Eigen::VectorXd saved_q, saved_dq, saved_ddq;
        saved_q.resize( ndof );
        saved_dq.resize( ndof );
        saved_ddq.resize( ndof );

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            saved_q[q_idx] = q[q_idx];
            saved_dq[q_idx] = dq[q_idx];
            saved_ddq[q_idx] = ddq[q_idx];
        }

        std::string effector_name = "effector";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        KinematicModel kin_model(robot_description_str, joint_names);

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

//        ros::Duration(1.0).sleep();

        //
        // reachability map for end-effector
        //

        
        r_map_.generate(kin_model, col_model, effector_name, ndof, lower_limit, upper_limit);

/*
        // TEST: reachability map
        int m_id = 3000;
        for (double x = -1.8; x < 1.8; x += 0.1) {
            for (double y = -1.8; y < 1.8; y += 0.1) {
                Eigen::VectorXd xs(2);
                xs(0) = x;
                xs(1) = y;
                double val = r_map_.getValue(xs);
                m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xs(0), xs(1), 0), 0, 0, 1, 1, 0.1*val, "base");
            }
        }
        markers_pub_.publish();
        ros::spinOnce();
        ros::Duration(0.01).sleep();
//        ros::Duration(1).sleep();
//        return;
//*/

        //
        // Tasks declaration
        //
        Task_JLC task_JLC(lower_limit, upper_limit, limit_range, max_trq);
        double activation_dist = 0.05;
        Task_COL task_COL(ndof, activation_dist, 10.0, kin_model, col_model);
        double activation_dist2 = 2.0;
        Task_COL task_COL2(ndof, activation_dist2, 10.0, kin_model, col_model);
        Task_HAND task_HAND(ndof, 3);
        Task_QA task_QA(ndof, lower_limit, upper_limit);

        RRTStar rrt(2, boost::bind(&TestDynamicModel::checkCollision, this, _1, links_fk, col_model),
                    boost::bind(&TestDynamicModel::costLine, this, _1, _2), boost::bind(&TestDynamicModel::sampleSpace, this, _1), 0.05, 0.2, 0.4 );

        RRTStar rrtQ5(5, boost::bind(&TestDynamicModel::checkCollisionQ5, this, _1, col_model, kin_model),
                        boost::bind(&TestDynamicModel::costLineQ5, this, _1, _2), boost::bind(&TestDynamicModel::sampleSpaceQ5, this, _1, lower_limit, upper_limit),
                        3.0/180.0*PI, 60.0/180.0*PI, 80.0/180.0*PI );

        // TEST: RRT (Q5)
        while (ros::ok()) {

            Eigen::VectorXd xs(ndof), xe(ndof);
            xs = q;
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                xe(q_idx) = randomUniform(lower_limit(q_idx), upper_limit(q_idx));
            }

            if (!rrt.isStateValid(xe)) {
                continue;
            }

            KDL::Frame T_B_E;
            kin_model.calculateFk(T_B_E, col_model->getLinkName(effector_idx), xe);
            publishTransform(T_B_E, "effector_dest");

            std::list<Eigen::VectorXd > path;
            rrtQ5.plan(xs, xe, 0.05, path);

            std::cout << "path.size(): " << path.size() << std::endl;
            if (path.size() == 0) {
                continue;
            }

            for (double f = 0.0; f < 1.0; f += 0.001) {
                Eigen::VectorXd x(ndof);
                getPointOnPath(path, f, x);

                std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
                // calculate forward kinematics for all links
                for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                    kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
                }

                publishJointState(x, joint_names);
                int m_id = 0;
                m_id = publishRobotModelVis(m_id, col_model, links_fk);
                markers_pub_.publish();

                ros::spinOnce();
                ros::Duration(0.01).sleep();
            }

            getchar();

            q = xe;
        }

        return;


/*
        // TEST: RRT
        while (ros::ok()) {

            Eigen::VectorXd xs(2), xe(2);
            xs(0) = randomUniform(0,2);
            xs(1) = randomUniform(-1,1);
            xe(0) = randomUniform(0,2);
            xe(1) = randomUniform(-1,1);

            if (!rrt.isStateValid(xs) || !rrt.isStateValid(xe)) {
                continue;
            }

            std::list<Eigen::VectorXd > path;
            rrt.plan(xs, xe, 0.05, path);

            int m_id = 100;
            m_id = rrt.addTreeMarker(markers_pub_, m_id);
            m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xe(0), xe(1), 0), 0, 0, 1, 1, 0.05, "base");
            for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
                KDL::Vector pos1((*it1)(0), (*it1)(1), 0), pos2((*it2)(0), (*it2)(1), 0);
                m_id = markers_pub_.addVectorMarker(m_id, pos1, pos2, 1, 1, 1, 1, 0.01, "base");
            }
            markers_pub_.addEraseMarkers(m_id, 2000);
            markers_pub_.publish();
            ros::spinOnce();
            ros::Duration(0.001).sleep();

            getchar();
        }

        return;
*/

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
//        r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(0,2), randomUniform(-1,1), 0));
        int loop_counter = 10000;
        ros::Rate loop_rate(500);
        bool pose_reached  = false;
        int try_idx = 50;
        std::list<Eigen::VectorXd > target_path;
        std::vector<SphereQ > spheres;
        std::list<Eigen::VectorXd > realized_path;

        std::vector<Eigen::VectorXd > eig_COL_vec(4);
        int eig_COL_vec_idx = 0;

        std::vector<Eigen::VectorXd > torque_COL_vec(4);
        int torque_COL_vec_idx = 0;

        Eigen::VectorXd const_torque(ndof);
        KDL::Twist diff_target;
        KDL::Frame r_HAND_start;

        while (ros::ok()) {

            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }
            // calculate inertia matrix for whole body
            dyn_model.computeM(q);

            if (loop_counter > 1500 || pose_reached) {
                if (pose_reached || try_idx > 15) {
                    if (try_idx > 15) {
                        std::cout << "Could not reach the pose" << std::endl;
                        publishJointState(q, joint_names);
                        int m_id = 0;
                        m_id = publishRobotModelVis(m_id, col_model, links_fk);
                        ros::Time last_time = ros::Time::now();
                    }
                    else {
                        std::cout << "Pose reached" << std::endl;
                        publishJointState(q, joint_names);
                        int m_id = 0;
                        m_id = publishRobotModelVis(m_id, col_model, links_fk);
                        ros::Time last_time = ros::Time::now();
                    }
                    bool pose_found = false;
                    Eigen::VectorXd xe(2);
                    // set new random target pose
                    for (int i = 0; i < 100; i++) {
                        r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(0,1.8), randomUniform(-1,1), 0));
                        xe(0) = r_HAND_target.p.x();
                        xe(1) = r_HAND_target.p.y();
                        if (rrt.isStateValid(xe)) {
                            pose_found = true;
                            break;
                        }
                    }
                    if (!pose_found) {
                        std::cout << "ERROR: could not find valid pose" << std::endl;
                        return;
                    }
                    publishTransform(r_HAND_target, "effector_dest");

                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        const_torque(q_idx) = 0.0;
                    }

                    r_map_.resetPenalty();
                    realized_path.clear();
                    spheres.clear();
                    penalty_points_.clear();

                    // get the current pose
                    Eigen::VectorXd xs(2);
                    xs(0) = links_fk[effector_idx].p.x();
                    xs(1) = links_fk[effector_idx].p.y();
                    std::list<Eigen::VectorXd > path;
                    rrt.plan(xs, xe, 0.05, path);
                    target_path = path;
                    r_HAND_start = links_fk[effector_idx];
                    diff_target = KDL::diff(r_HAND_start, r_HAND_target, 1.0);

                    // visualize the planned graph
                    int m_id = 100;
//                    m_id = rrt.addTreeMarker(markers_pub_, m_id);
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xe(0), xe(1), 0), 0, 0, 1, 1, 0.05, "base");
                    for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
                        KDL::Vector pos1((*it1)(0), (*it1)(1), 0), pos2((*it2)(0), (*it2)(1), 0);
                        m_id = markers_pub_.addVectorMarker(m_id, pos1, pos2, 1, 1, 1, 1, 0.02, "base");
                    }

                    markers_pub_.addEraseMarkers(m_id, 2000);
                    markers_pub_.publish();
                    ros::spinOnce();
                    ros::Duration(0.01).sleep();

                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        saved_q[q_idx] = q[q_idx];
                        saved_dq[q_idx] = dq[q_idx];
                        saved_ddq[q_idx] = ddq[q_idx];
                    }
                    try_idx = 0;
                    getchar();
                }
                else {
                    try_idx++;

//                    for (std::list<Eigen::VectorXd >::const_iterator it = realized_path.begin(); it != realized_path.end(); it++) {
//                        SphereQ sq( (*it), 60.0/180.0 * PI);
//                        spheres.push_back(sq);
//                    }
                    realized_path.clear();

                    if (try_idx % 2 == 1) {
                        double angle = diff_target.rot.Norm();
                        diff_target.rot = (angle - 360.0/180.0*PI) * diff_target.rot / angle;
                        std::cout << "trying other rotation..." << std::endl;
                    }
                    else {
//                        if (target_path.size() > 2) {
                        for (double f = 0.1; f <= 0.9; f += 0.01) {
                            Eigen::VectorXd pt(2);
                            getPointOnPath(target_path, f, pt);
                            r_map_.addPenalty( pt );
                        }
                            std::cout << "added penalty " << std::endl;
//                        }

                        Eigen::VectorXd xe(2);
                        xe(0) = r_HAND_target.p.x();
                        xe(1) = r_HAND_target.p.y();

                        // get the current pose
                        // calculate forward kinematics for all links
                        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                            kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), saved_q);
                        }

                        std::cout << "replanning the path..." << std::endl;
                        Eigen::VectorXd xs(2);
                        xs(0) = links_fk[effector_idx].p.x();
                        xs(1) = links_fk[effector_idx].p.y();
                        std::list<Eigen::VectorXd > path;
                        rrt.plan(xs, xe, 0.05, path);
                        target_path = path;

                        // visualize the planned graph
                        int m_id = 100;
//                        m_id = rrt.addTreeMarker(markers_pub_, m_id);
                        m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xe(0), xe(1), 0), 0, 0, 1, 1, 0.05, "base");
                        for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
                            KDL::Vector pos1((*it1)(0), (*it1)(1), 0), pos2((*it2)(0), (*it2)(1), 0);
                            m_id = markers_pub_.addVectorMarker(m_id, pos1, pos2, 1, 1, 1, 1, 0.02, "base");
                        }

                        markers_pub_.addEraseMarkers(m_id, 2000);
                        markers_pub_.publish();
                        ros::spinOnce();
                        ros::Duration(0.01).sleep();
                    }

                    // another try
                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        q[q_idx] = saved_q[q_idx];
                        dq[q_idx] = saved_dq[q_idx];
                        ddq[q_idx] = saved_ddq[q_idx];
                    }

/*
                    if (target_path.size() > 2) {
                        // add penalty for the last path
                        for (std::list<Eigen::VectorXd >::const_iterator it = ++target_path.begin(); it != --target_path.end(); it++) {
                            penalty_points_.push_back( (*it) );
                        }
                    }
                    std::cout << "penalty_points " << penalty_points_.size() << std::endl;

                    Eigen::VectorXd xe(2);
                    xe(0) = r_HAND_target.p.x();
                    xe(1) = r_HAND_target.p.y();

                    // get the current pose
                    // calculate forward kinematics for all links
                    for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                        kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
                    }

                    Eigen::VectorXd xs(2);
                    xs(0) = links_fk[effector_idx].p.x();
                    xs(1) = links_fk[effector_idx].p.y();
                    std::list<Eigen::VectorXd > path;
                    rrt.plan(xs, xe, 0.05, path);
                    target_path = path;

                    // visualize the planned graph
                    int m_id = 100;
                    m_id = rrt.addTreeMarker(markers_pub_, m_id);
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xe(0), xe(1), 0), 0, 0, 1, 1, 0.05, "base");
                    for (std::list<Eigen::VectorXd >::const_iterator it1 = path.begin(), it2=++path.begin(); it2 != path.end(); it1++, it2++) {
                        KDL::Vector pos1((*it1)(0), (*it1)(1), 0), pos2((*it2)(0), (*it2)(1), 0);
                        m_id = markers_pub_.addVectorMarker(m_id, pos1, pos2, 1, 1, 1, 1, 0.02, "base");
                    }

                    markers_pub_.addEraseMarkers(m_id, 2000);
                    markers_pub_.publish();
                    ros::spinOnce();
                    ros::Duration(0.001).sleep();
*/
                }

                pose_reached  = false;

                loop_counter = 0;
            }
            loop_counter += 1;

            if (realized_path.size() == 0) {
                if ((q-saved_q).norm() > 10.0/180.0*PI) {
                    realized_path.push_back(q);
                }
            }
            else {
                if ((q-realized_path.back()).norm() > 10.0/180.0*PI) {
                    realized_path.push_back(q);
                }
            }


            //
            // joint limit avoidance
            //
            Eigen::VectorXd torque_JLC(ndof);
            Eigen::MatrixXd N_JLC(ndof, ndof);
            task_JLC.compute(q, dq, dyn_model.I, torque_JLC, N_JLC);

            //
            // collision constraints
            //
            std::vector<CollisionInfo> link_collisions;
            getCollisionPairs(col_model, links_fk, activation_dist, link_collisions);

            Eigen::VectorXd torque_COL(ndof);
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                torque_COL[q_idx] = 0.0;
            }
            Eigen::MatrixXd N_COL(Eigen::MatrixXd::Identity(ndof, ndof));

            task_COL.compute(q, dq, dyn_model.invI, links_fk, link_collisions, torque_COL, N_COL);

            //
            // effector task
            //
            Eigen::VectorXd pt(2);
            double f_path = static_cast<double >(loop_counter)/800.0;
            if (f_path > 1.0) {
                f_path = 1.0;
            }
            getPointOnPath(target_path, f_path, pt);
            markers_pub_.addSinglePointMarker(50, KDL::Vector(pt(0), pt(1), 0), 1, 0, 0, 1, 0.2, "base");

            Eigen::VectorXd torque_HAND(ndof);
            Eigen::MatrixXd N_HAND(ndof, ndof);

            KDL::Frame T_B_E = links_fk[effector_idx];
            KDL::Frame r_HAND_current = T_B_E;

            KDL::Rotation target_rot = KDL::addDelta(r_HAND_start, diff_target, f_path).M;

            KDL::Frame target_pos(target_rot, KDL::Vector(pt(0), pt(1), 0));
            KDL::Twist diff = KDL::diff(r_HAND_current, target_pos, 1.0);
//            KDL::Twist diff = KDL::diff(r_HAND_current, r_HAND_target, 1.0);
            Eigen::VectorXd r_HAND_diff(3);
            r_HAND_diff[0] = diff[0];
            r_HAND_diff[1] = diff[1];
            r_HAND_diff[2] = diff[5];

            KDL::Twist diff_goal = KDL::diff(r_HAND_current, r_HAND_target, 1.0);

            if (diff_goal.vel.Norm() < 0.06 && diff.rot.Norm() < 5.0/180.0 * PI) {
//                std::cout << "Pose reached " << diff.vel.Norm() << " " << diff.rot.Norm() << std::endl;
                pose_reached  = true;
                continue;
            }

            Eigen::VectorXd Kc(3);
            Kc[0] = 10.0;
            Kc[1] = 10.0;
            Kc[2] = 1.00;
            Eigen::VectorXd Dxi(3);
            Dxi[0] = 0.7;
            Dxi[1] = 0.7;
            Dxi[2] = 0.7;

            kin_model.getJacobian(J_r_HAND_6, effector_name, q);

            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                J_r_HAND(0, q_idx) = J_r_HAND_6(0, q_idx);
                J_r_HAND(1, q_idx) = J_r_HAND_6(1, q_idx);
                J_r_HAND(2, q_idx) = J_r_HAND_6(5, q_idx);
            }

            task_HAND.compute(r_HAND_diff, Kc, Dxi, J_r_HAND, dq, dyn_model.invI, torque_HAND, N_HAND);

            Eigen::VectorXd torque_QA(ndof);
            Eigen::MatrixXd N_QA(ndof, ndof);
            task_QA.compute(q, dq, dyn_model.I, spheres, torque_QA, N_QA);

            //
            // repulsive field
            //
            std::vector<CollisionInfo> link_collisions2;
            getRepulsiveForces(col_model, links_fk, activation_dist2, r_HAND_target.p, link_collisions2);

            Eigen::VectorXd torque_COL2(ndof);
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                torque_COL2[q_idx] = 0.0;
            }
            Eigen::MatrixXd N_COL2(Eigen::MatrixXd::Identity(ndof, ndof));

            task_COL2.compute(q, dq, dyn_model.invI, links_fk, link_collisions2, torque_COL2, N_COL2);


//            torque = torque_HAND + N_HAND * (torque_JLC + N_JLC.transpose() * torque_COL);// + N_HAND.transpose() * torque_QA)));

//            torque = torque_JLC + N_JLC.transpose() * (torque_HAND + N_HAND.transpose() * torque_COL2);
            torque = torque_JLC + N_JLC.transpose() * (torque_COL + (N_COL.transpose() * (torque_HAND)));// + N_HAND.transpose() * torque_COL2)));
//            torque = torque_JLC + N_JLC.transpose() * (torque_COL + (N_COL.transpose() * (torque_HAND + N_HAND.transpose() * torque_QA)));
//            torque = torque_JLC + N_JLC.transpose() * (torque_COL + (N_COL.transpose() * (torque_COL2 + N_COL2.transpose() * torque_HAND)));

            torque_COL_vec[torque_COL_vec_idx] = torque_COL;
            torque_COL_vec_idx = (torque_COL_vec_idx + 1) % torque_COL_vec.size();



//            torque = torque_JLC + N_JLC.transpose() * torque_COL;
//            torque = torque_JLC + N_JLC.transpose() * torque_HAND;
//            torque = torque_HAND;

            // simulate one step
            dyn_model.accel(ddq, q, dq, torque);
            float time_d = 0.01;
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                dq[q_idx] += ddq[q_idx] * time_d;
                q[q_idx] += dq[q_idx] * time_d;
            }

            // publish markers and robot state with limited rate
            ros::Duration time_elapsed = ros::Time::now() - last_time;
            if (time_elapsed.toSec() > 0.05) {
                publishJointState(q, joint_names);
                int m_id = 0;
                m_id = publishRobotModelVis(m_id, col_model, links_fk);
                markers_pub_.publish();
                ros::Time last_time = ros::Time::now();
            }
            ros::spinOnce();
//            loop_rate.sleep();
        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}

