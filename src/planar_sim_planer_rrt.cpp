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

class RRTStar {
public:
    RRTStar(boost::function<bool(const Eigen::VectorXd &x)> collision_func, boost::function<double(const Eigen::VectorXd &x, const Eigen::VectorXd &y)> costLine_func) :
        collision_func_(collision_func),
        costLine_func_(costLine_func)
    {
    }

    bool isStateValid(const Eigen::VectorXd &x) const {
        return !collision_func_(x);
    }

    bool sampleFree(Eigen::VectorXd &sample_free) const {
        Eigen::VectorXd x(2);
        for (int i=0; i < 100; i++) {
            x(0) = randomUniform(0,2);
            x(1) = randomUniform(-1,1);
            if (isStateValid(x)) {
                sample_free = x;
                return true;
            }
        }
        return false;
    }

    int nearest(const Eigen::VectorXd &x) const {
        double min_dist = -1.0;
        int min_idx = -1;
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - x).norm();
            if (min_idx < 0 || dist < min_dist) {
                min_dist = dist;
                min_idx = v_it->first;
            }
        }
        return min_idx;
    }

    void steer(const Eigen::VectorXd &x_from, const Eigen::VectorXd &x_to, double steer_dist, Eigen::VectorXd &x) const {
        Eigen::VectorXd v = x_to - x_from;
        if (v.norm() <= steer_dist) {
            x = x_to;
        }
        else {
            x = x_from + steer_dist * v / v.norm();
        }
    }

    bool collisionFree(const Eigen::VectorXd &x_from, const Eigen::VectorXd &x_to) const {
        double step = 0.05;
        Eigen::VectorXd v = x_to - x_from;
        double dist = v.norm();
        v = v / dist;
        double progress = 0.0;
        while (true) {
            progress += step;
            bool end = false;
            if (progress > dist) {
                progress = dist;
                end = true;
            }
            Eigen::VectorXd x = x_from + v * progress;
            if (!isStateValid(x)) {
                return false;
            }
            if (end) {
                break;
            }
        }
        return true;
    }

    void near(const Eigen::VectorXd &x, double near_dist, std::list<int > &q_near_idx_list) const {
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - x).norm();
            if (dist <= near_dist) {
                q_near_idx_list.push_back(v_it->first);
            }
        }
    }

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const {
        return costLine_func_(x1, x2);
    }

    double costLine(int x1_idx, int x2_idx) const {
        return costLine(V_.find(x1_idx)->second, V_.find(x2_idx)->second);
    }


    double cost(int q_idx) const {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            return 0.0;
        }
        int q_parent_idx = e_it->second;
        return costLine(q_idx, q_parent_idx) + cost(q_parent_idx);
    }

    void getPath(int q_idx, std::list<int > &path) {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            path.clear();
            path.push_back(q_idx);
        }
        else {
            int q_parent_idx = e_it->second;
            getPath(q_parent_idx, path);
            path.push_back(q_idx);
        }
    }

    void plan(const Eigen::VectorXd &start, const Eigen::VectorXd &goal, std::list<Eigen::VectorXd > &path) {
        double goal_tolerance = 0.05;

        V_.clear();
        E_.clear();
        path.clear();

        double steer_dist = 0.2;
        double near_dist = 0.4;
        int q_new_idx = 0;
        V_[0] = start;

        for (int step = 0; step < 1000; step++) {
            bool sample_goal = randomUniform(0,1) < 0.05;
            Eigen::VectorXd q_rand(2);

            if (sample_goal) {
                q_rand = goal;
            }
            else {
                if (!sampleFree(q_rand)) {
                    std::cout << "ERROR: RRTStar::plan: could not sample free space" << std::endl;
                    return;
                }
            }

            int q_nearest_idx = nearest(q_rand);
            const Eigen::VectorXd &q_nearest = V_.find(q_nearest_idx)->second;
            Eigen::VectorXd q_new(2);
            steer(q_nearest, q_rand, steer_dist, q_new);

            if (collisionFree(q_nearest, q_new)) {

                bool isGoal = (q_new - goal).norm() < goal_tolerance;

                std::list<int > q_near_idx_list;
                near(q_new, near_dist, q_near_idx_list);
                double min_cost = cost(q_nearest_idx);
                int min_idx = q_nearest_idx;                
                for (std::list<int >::const_iterator qi_it = q_near_idx_list.begin(); qi_it != q_near_idx_list.end(); qi_it++) {
                    int q_idx = *qi_it;
                    double c = cost(q_idx);
                    if (min_idx == -1 || min_cost > c && collisionFree(V_.find(q_idx)->second, q_new)) {
                        min_idx = q_idx;
                        min_cost = c;
                    }
                }

                q_new_idx++;
                V_[q_new_idx] = q_new;
                E_[q_new_idx] = min_idx;


                double cost_q_new = cost(q_new_idx);
                for (std::list<int >::const_iterator qi_it = q_near_idx_list.begin(); qi_it != q_near_idx_list.end(); qi_it++) {
                    int q_near_idx = *qi_it;
                    Eigen::VectorXd q_near = V_.find(q_near_idx)->second;
                    if (cost_q_new + costLine(q_new, q_near) < cost(q_near_idx)) {
                        bool col_free = collisionFree(q_new, q_near);
                        if (col_free) {
                                int q_parent_idx = E_[q_near_idx];
                                E_[q_near_idx] = q_new_idx;
                        }
                    }
                }
            }
        }

        double min_cost = 0.0;
        int min_goal_idx = -1;
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - goal).norm();
            double c = cost(v_it->first);
            if (dist < goal_tolerance && (min_goal_idx < 0 || min_cost > c)) {
                min_cost = c;
                min_goal_idx = v_it->first;
            }
        }

        std::list<int > idx_path;
        getPath(min_goal_idx, idx_path);
        for (std::list<int >::const_iterator p_it = idx_path.begin(); p_it != idx_path.end(); p_it++) {
            path.push_back(V_.find(*p_it)->second);
        }
    }

    int addTreeMarker(MarkerPublisher &markers_pub, int m_id) const {
        std::vector<std::pair<KDL::Vector, KDL::Vector > > vec_arr;
        for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
            const Eigen::VectorXd &x1 = V_.find(e_it->first)->second;
            const Eigen::VectorXd &x2 = V_.find(e_it->second)->second;
            KDL::Vector pos1(x1(0), x1(1), 0), pos2(x2(0), x2(1), 0);
            vec_arr.push_back( std::make_pair(pos1, pos2) );
            m_id = markers_pub.addVectorMarker(m_id, pos1, pos2, 0, 0.7, 0, 0.5, 0.01, "base");
        }

        const Eigen::VectorXd &xs = V_.find(0)->second;
        KDL::Vector pos(xs(0), xs(1), 0);
        m_id = markers_pub.addSinglePointMarker(m_id, pos, 0, 1, 0, 1, 0.05, "base");
        return m_id;
    }

protected:
    boost::function<bool(const Eigen::VectorXd &x)> collision_func_;
    boost::function<double(const Eigen::VectorXd &x, const Eigen::VectorXd &y)> costLine_func_;
    std::map<int, Eigen::VectorXd > V_;
    std::map<int, int > E_;
};

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

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const ReachabilityMap &r_map) const {
        return (x1-x2).norm() * (2.0 - r_map.getValue(x1) - r_map.getValue(x2));
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

        ReachabilityMap r_map(0.1, 2);
        r_map.generate(kin_model, col_model, effector_name, ndof, lower_limit, upper_limit);

//*
        // TEST: reachability map
        int m_id = 3000;
        for (double x = -1.8; x < 1.8; x += 0.1) {
            for (double y = -1.8; y < 1.8; y += 0.1) {
                Eigen::VectorXd xs(2);
                xs(0) = x;
                xs(1) = y;
                double val = r_map.getValue(xs);
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
        Task_COL task_COL(ndof, activation_dist, kin_model, col_model);

        Task_HAND task_HAND(ndof, 3);

        RRTStar rrt( boost::bind(&TestDynamicModel::checkCollision, this, _1, links_fk, col_model), boost::bind(&TestDynamicModel::costLine, this, _1, _2, r_map) );

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
            rrt.plan(xs, xe, path);

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


        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(0,2), randomUniform(-1,1), 0));
        int loop_counter = 0;
        ros::Rate loop_rate(500);
        bool pose_reached  = false;
        int try_idx = 0;
        while (ros::ok()) {

            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                kin_model.calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), q);
            }
            // calculate inertia matrix for whole body
            dyn_model.computeM(q);


            if (loop_counter > 1000) {
                if (pose_reached || try_idx > 40) {
                    if (try_idx > 40) {
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

                    // get the current pose
                    Eigen::VectorXd xs(2);
                    xs(0) = links_fk[effector_idx].p.x();
                    xs(1) = links_fk[effector_idx].p.y();
                    std::list<Eigen::VectorXd > path;
                    rrt.plan(xs, xe, path);

                    // visualize the planned graph
                    int m_id = 100;
                    m_id = rrt.addTreeMarker(markers_pub_, m_id);
                    m_id = markers_pub_.addSinglePointMarker(m_id, KDL::Vector(xe(0), xe(1), 0), 0, 0, 1, 1, 0.05, "base");
                    markers_pub_.addEraseMarkers(m_id, 2000);
                    markers_pub_.publish();
                    ros::spinOnce();
                    ros::Duration(0.001).sleep();

                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        saved_q[q_idx] = q[q_idx];
                        saved_dq[q_idx] = dq[q_idx];
                        saved_ddq[q_idx] = ddq[q_idx];
                    }
                    try_idx = 0;
                    getchar();
                }
                else {
                    // another try
                    for (int q_idx = 0; q_idx < ndof; q_idx++) {
                        q[q_idx] = saved_q[q_idx];
                        dq[q_idx] = saved_dq[q_idx];
                        ddq[q_idx] = saved_ddq[q_idx];
                    }
                    try_idx++;
                }

                pose_reached  = false;

                loop_counter = 0;
            }
            loop_counter += 1;


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

            if (diff.vel.Norm() < 0.06 && diff.rot.Norm() < 5.0/180.0 * PI) {
//                std::cout << "Pose reached " << diff.vel.Norm() << " " << diff.rot.Norm() << std::endl;
                pose_reached  = true;
                continue;
            }

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

            task_HAND.compute(r_HAND_diff, Kc, Dxi, J_r_HAND, dq, dyn_model.invI, torque_HAND);

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

