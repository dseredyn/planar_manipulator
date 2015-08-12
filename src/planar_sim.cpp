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

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

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
        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Capsule());
        boost::shared_ptr<self_collision::Capsule > cap = boost::static_pointer_cast<self_collision::Capsule >(pcol->geometry);
        cap->radius = 0.2;
        cap->length = 0.3;
        pcol->origin = KDL::Frame(KDL::Vector(1, 0.5, 0));
        col_array.push_back(pcol);
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
            q[q_idx] = 0.1;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
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

        //
        // Tasks declaration
        //
        Task_JLC task_JLC(lower_limit, upper_limit, limit_range, max_trq);
        double activation_dist = 0.05;
        Task_COL task_COL(ndof, activation_dist, 10.0, kin_model, col_model);
        Task_HAND task_HAND(ndof, 3);

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 10000;
        ros::Rate loop_rate(100);
        while (ros::ok()) {

            if (loop_counter > 500) {
                r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(0,2), randomUniform(-1,1), 0));

                publishTransform(r_HAND_target, "effector_dest");
                loop_counter = 0;
            }
            loop_counter += 1;

            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }
            dyn_model.computeM(q);

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
            Eigen::VectorXd torque_HAND(ndof);

            KDL::Frame T_B_E = links_fk[effector_idx];
            KDL::Frame r_HAND_current = T_B_E;
            KDL::Twist diff = KDL::diff(r_HAND_current, r_HAND_target, 1.0);
            Eigen::VectorXd r_HAND_diff(3);
            r_HAND_diff[0] = diff[0];
            r_HAND_diff[1] = diff[1];
            r_HAND_diff[2] = diff[5];

            Eigen::VectorXd Kc(3);
            Kc[0] = 10.0;
            Kc[1] = 10.0;
            Kc[2] = 1.0;
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

            Eigen::MatrixXd N_HAND(ndof, ndof);
            task_HAND.compute(r_HAND_diff, Kc, Dxi, J_r_HAND, dq, dyn_model.invI, torque_HAND, N_HAND);

            torque = torque_JLC + N_JLC.transpose() * (torque_COL + (N_COL.transpose() * torque_HAND));
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
            loop_rate.sleep();
        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}


