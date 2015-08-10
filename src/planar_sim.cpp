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

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#define RESTRICT_ALLOC
#define UNRESTRICT_ALLOC
#endif

class Task_HAND {
public:
    Task_HAND(int ndof, int dim) :
        ndof_(ndof),
        dim_(dim),
        wrench_(dim),
        JT(ndof_, dim_),
        tmpNK_(ndof_, dim_),
        A(dim_, dim_),
        Q(dim_, dim_),
        tmpKK_(dim_, dim_),
        tmpKK2_(dim_, dim_),
        Dc(dim_, dim_),
        K0(dim_),
        tmpK_(dim_),
        wrench_tmp(dim_)
    {
    }

    ~Task_HAND() {
    }

    void compute(const Eigen::VectorXd &T_diff, const Eigen::VectorXd &Kc, const Eigen::VectorXd &Dxi, const Eigen::MatrixXd &J, const Eigen::VectorXd &dq, const Eigen::MatrixXd &invI,
                    Eigen::VectorXd &torque)
    {
            for (int dim_idx = 0; dim_idx < dim_; dim_idx++) {
                wrench_[dim_idx] = Kc[dim_idx] * T_diff[dim_idx];
            }
            torque = J.transpose() * wrench_;

            // code form cartesian_impedance.h
            JT = J.transpose();
            tmpNK_.noalias() = J * invI;
            A.noalias() = tmpNK_ * JT;
            luKK_.compute(A);
            A = luKK_.inverse();

            tmpKK_ = Kc.asDiagonal();
            UNRESTRICT_ALLOC;
            es_.compute(tmpKK_, A);
            RESTRICT_ALLOC;
            K0 = es_.eigenvalues();
            luKK_.compute(es_.eigenvectors());
            Q = luKK_.inverse();

            tmpKK_ = Dxi.asDiagonal();
            Dc.noalias() = Q.transpose() * tmpKK_;
            tmpKK_ = K0.cwiseSqrt().asDiagonal();
            tmpKK2_.noalias() = Dc *  tmpKK_;
            Dc.noalias() = tmpKK2_ * Q;
            tmpK_.noalias() = J * dq;
            // TODO: check if 2.0* is ok
            wrench_tmp.noalias() = 2.0 * Dc * tmpK_;
            torque.noalias() -= JT * wrench_tmp;
    }

protected:
    int ndof_, dim_;
    Eigen::VectorXd wrench_;
    Eigen::MatrixXd JT;
    Eigen::MatrixXd tmpNK_;
    Eigen::MatrixXd A;
    Eigen::PartialPivLU<Eigen::MatrixXd> luKK_;
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es_;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd tmpKK_;
    Eigen::MatrixXd tmpKK2_;
    Eigen::MatrixXd Dc;
    Eigen::VectorXd K0;
    Eigen::VectorXd tmpK_;
    Eigen::VectorXd wrench_tmp;
};

class Task_JLC {
public:
    Task_JLC(const Eigen::VectorXd &lower_limit, const Eigen::VectorXd &upper_limit, const Eigen::VectorXd &limit_range, const Eigen::VectorXd &max_trq) :
        q_length_(lower_limit.innerSize()),
        lower_limit_(lower_limit),
        upper_limit_(upper_limit),
        limit_range_(limit_range),
        max_trq_(max_trq),
        activation_JLC_(q_length_),
        k_(q_length_),
        k0_(q_length_),
        q_(q_length_, q_length_), d_(q_length_, q_length_),
        J_JLC_(q_length_, q_length_),
        tmpNN_(q_length_, q_length_)
    {
    }

    ~Task_JLC() {
    }

    double jointLimitTrq(double hl, double ll, double ls,
        double r_max, double q, double &out_limit_activation) {
        if (q > (hl - ls)) {
            out_limit_activation = fabs((q - hl + ls) / ls);
            return -1 * ((q - hl + ls) / ls) * ((q - hl + ls) / ls) * r_max;
        } else if (q < (ll + ls)) {
            out_limit_activation = fabs((ll + ls - q) / ls);
            return ((ll + ls - q) / ls) * ((ll + ls - q) / ls) * r_max;
        } else {
            out_limit_activation = 0.0;
            return 0.0;
        }
    }

    void compute(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &I, Eigen::VectorXd &torque, Eigen::MatrixXd &N) {
            // code from joint_limit_avoidance.cpp
            for (int q_idx = 0; q_idx < q_length_; q_idx++) {
                torque(q_idx) = jointLimitTrq(upper_limit_[q_idx],
                                           lower_limit_[q_idx], limit_range_[q_idx], max_trq_[q_idx],
                                           q[q_idx], activation_JLC_[q_idx]);
                activation_JLC_[q_idx] *= 10.0;
                if (activation_JLC_[q_idx] > 1.0) {
                    activation_JLC_[q_idx] = 1.0;
                }

                if (fabs(torque(q_idx)) > 0.001) {
                    k_(q_idx) = max_trq_[q_idx]/limit_range_[q_idx];
                } else {
                    k_(q_idx) = 0.001;
                }
            }

            tmpNN_ = k_.asDiagonal();
            es_.compute(tmpNN_, I);
            q_ = es_.eigenvectors().inverse();
            k0_ = es_.eigenvalues();

            tmpNN_ = k0_.cwiseSqrt().asDiagonal();

            d_.noalias() = 2.0 * q_.adjoint() * 0.7 * tmpNN_ * q_;

            torque.noalias() -= d_ * dq;

            // calculate jacobian (the activation function)
            J_JLC_ = activation_JLC_.asDiagonal();
            N = Eigen::MatrixXd::Identity(q_length_, q_length_) - (J_JLC_.transpose() * J_JLC_);
    }

protected:
    int q_length_;
    Eigen::VectorXd lower_limit_;
    Eigen::VectorXd upper_limit_;
    Eigen::VectorXd limit_range_;
    Eigen::VectorXd max_trq_;
    Eigen::VectorXd activation_JLC_;
    Eigen::VectorXd k_;
    Eigen::VectorXd k0_;
    Eigen::MatrixXd q_, d_;
    Eigen::MatrixXd J_JLC_;
    Eigen::MatrixXd tmpNN_;
    Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es_;
};

class CollisionInfo {
public:
    int link1_idx;
    int link2_idx;
    KDL::Vector p1_B;
    KDL::Vector p2_B;
    double dist;
    KDL::Vector n1_B;
    KDL::Vector n2_B;
};

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Publisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

public:
    TestDynamicModel() :
        PI(3.141592653589793)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/velma_markers", 10);
    }

    ~TestDynamicModel() {
    }

    double randomUniform(double min, double max) {
        return min + (max - min) * (double)rand() / (double)RAND_MAX;
    }

    void distancePoints(const KDL::Vector &pt1, const KDL::Vector &pt2, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        distance = (pt1-pt2).Norm();
        p_pt1 = pt1;
        p_pt2 = pt2;
    }

    void distanceLinePoint(const KDL::Vector &lineA, const KDL::Vector &lineB, const KDL::Vector &pt, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        KDL::Vector v = lineB - lineA;
        double ta = KDL::dot(v, lineA);
        double tb = KDL::dot(v, lineB);
        double tpt = KDL::dot(v, pt);
        if (tpt <= ta) {
            distance = (lineA-pt).Norm();
            p_pt1 = lineA;
            p_pt2 = pt;
        }
        else if (tpt >= tb) {
            distance = (lineB-pt).Norm();
            p_pt1 = lineB;
            p_pt2 = pt;
        }
        else {
            KDL::Vector n(v.y(), -v.x(), v.z());
            n.Normalize();
            double diff = KDL::dot(n, lineA) - KDL::dot(n, pt);
            distance = fabs(diff);
            p_pt1 = pt + (diff * n);
            p_pt2 = pt;
        }
    }

    void distancePointLine(const KDL::Vector &pt, const KDL::Vector &lineA, const KDL::Vector &lineB, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        distanceLinePoint(lineA, lineB, pt, distance, p_pt2, p_pt1);
    }

    void distanceLines(const KDL::Vector &l1A, const KDL::Vector &l1B, const KDL::Vector &l2A, const KDL::Vector &l2B, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        double x1 = l1A.x(), x2 = l1B.x(), x3 = l2A.x(), x4 = l2B.x();
        double y1 = l1A.y(), y2 = l1B.y(), y3 = l2A.y(), y4 = l2B.y();
        double denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
        // check if the lines cross
        if (denom != 0.0) {
            double xi = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / denom;
            double yi = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / denom;
            if (((xi >= x1 && xi <= x2) || (xi >= x2 && xi <= x1)) &&
                ((yi >= y1 && yi <= y2) || (yi >= y2 && yi <= y1)) &&
                ((xi >= x3 && xi <= x4) || (xi >= x4 && xi <= x3)) &&
                ((yi >= y3 && yi <= y4) || (yi >= y4 && yi <= y3))) {
                distance = 0.0;
                p_pt1 = KDL::Vector(xi, yi, 0.0);
                p_pt2 = KDL::Vector(xi, yi, 0.0);
                return;
            }
        }
        double tmp_distance[4];
        KDL::Vector tmp_p_pt1[4], tmp_p_pt2[4];
        distanceLinePoint(l1A, l1B, l2A, tmp_distance[0], tmp_p_pt1[0], tmp_p_pt2[0]);
        distanceLinePoint(l1A, l1B, l2B, tmp_distance[1], tmp_p_pt1[1], tmp_p_pt2[1]);
        distancePointLine(l1A, l2A, l2B, tmp_distance[2], tmp_p_pt1[2], tmp_p_pt2[2]);
        distancePointLine(l1B, l2A, l2B, tmp_distance[3], tmp_p_pt1[3], tmp_p_pt2[3]);
        
        double min_dist = 10000000.0;
        int min_idx = 0;
        for (int idx = 0; idx < 4; idx++) {
            if (tmp_distance[idx] < min_dist) {
                min_dist = tmp_distance[idx];
                min_idx = idx;
            }
        }
        distance = tmp_distance[min_idx];
        p_pt1 = tmp_p_pt1[min_idx];
        p_pt2 = tmp_p_pt2[min_idx];
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
        for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
            boost::shared_ptr<self_collision::Link> link = col_model->links_[l_idx];
            KDL::Frame T_B_L = T[l_idx];
            for (self_collision::Link::VecPtrCollision::const_iterator it = link->collision_array.begin(); it != link->collision_array.end(); it++) {
                KDL::Frame T_B_O = T_B_L * (*it)->origin;
                if ((*it)->geometry->type == self_collision::Geometry::CONVEX) {
                    // TODO
                }
                else if ((*it)->geometry->type == self_collision::Geometry::SPHERE) {
                    self_collision::Sphere *sphere = static_cast<self_collision::Sphere* >((*it)->geometry.get());
                    m_id = publishSinglePointMarker(markers_pub_, m_id, T_B_O.p, 0, 1, 0, sphere->radius*2, "base");
                }
                else if ((*it)->geometry->type == self_collision::Geometry::CAPSULE) {
                    self_collision::Capsule *capsule = static_cast<self_collision::Capsule* >((*it)->geometry.get());
                    m_id = publishCapsule(markers_pub_, m_id, T_B_O, capsule->length, capsule->radius, "base");
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
        int effector_idx = 0;
        for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
            if (col_model->links_[l_idx]->name == effector_name) {
                effector_idx = l_idx;
                break;
            }
        }

        //
        // kinematic model
        //
        KinematicModel kin_model(robot_description_str, joint_names);

        KinematicModel::Jacobian J_r_HAND_6, J_r_HAND;
        J_r_HAND_6.resize(6, ndof);
        J_r_HAND.resize(3, ndof);

        std::vector<KDL::Frame > links_fk(col_model->link_count_);

        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof), limit_range(ndof), max_trq(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {
            for (std::vector<self_collision::Joint>::const_iterator j_it = col_model->joints_.begin(); j_it != col_model->joints_.end(); j_it++) {
                if ( (*name_it) == j_it->name_) {
                    lower_limit[q_idx] = j_it->lower_limit_;
                    upper_limit[q_idx] = j_it->upper_limit_;
                    break;
                }
            }
            limit_range[q_idx] = 10.0/180.0*PI;
            max_trq[q_idx] = 10.0;
        }

        //
        // Tasks declaration
        //
        Task_JLC task_JLC(lower_limit, upper_limit, limit_range, max_trq);
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
            for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
                kin_model.calculateFk(links_fk[l_idx], col_model->links_[l_idx]->name, q);
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
            double activation_dist = 0.05;
            // self collision
            for (self_collision::CollisionModel::CollisionPairs::const_iterator it = col_model->enabled_collisions.begin(); it != col_model->enabled_collisions.end(); it++) {
                int link1_idx = it->first;
                int link2_idx = it->second;
                boost::shared_ptr<self_collision::Link> link1 = col_model->links_[link1_idx];
                boost::shared_ptr<self_collision::Link> link2 = col_model->links_[link2_idx];
                KDL::Frame T_B_L1 = links_fk[link1_idx];
                KDL::Frame T_B_L2 = links_fk[link2_idx];

                for (self_collision::Link::VecPtrCollision::const_iterator col1 = link1->collision_array.begin(); col1 != link1->collision_array.end(); col1++) {
                    for (self_collision::Link::VecPtrCollision::const_iterator col2 = link2->collision_array.begin(); col2 != link2->collision_array.end(); col2++) {
                        KDL::Frame T_B_C1 = T_B_L1 * (*col1)->origin;
                        KDL::Frame T_B_C2 = T_B_L2 * (*col2)->origin;
                        boost::shared_ptr< self_collision::Geometry > geom1 = (*col1)->geometry;
                        boost::shared_ptr< self_collision::Geometry > geom2 = (*col2)->geometry;

                        double c_dist = (T_B_C1.p - T_B_C2.p).Norm();
                        double dist = 1000000.0;
                        double radius1, radius2;
                        KDL::Vector p1_B, p2_B;
                        if (geom1->type == self_collision::Geometry::CAPSULE && geom2->type == self_collision::Geometry::CAPSULE) {
                            self_collision::Capsule *capsule1 = static_cast<self_collision::Capsule* >(geom1.get());
                            self_collision::Capsule *capsule2 = static_cast<self_collision::Capsule* >(geom2.get());
                            radius1 = capsule1->radius;
                            radius2 = capsule2->radius;
                            if (c_dist - radius1 - radius2 - capsule1->length/2.0 - capsule2->length/2.0 > activation_dist) {
                                continue;
                            }
                            KDL::Vector line1A = T_B_C1 * KDL::Vector(0, -capsule1->length/2, 0);
                            KDL::Vector line1B = T_B_C1 * KDL::Vector(0, capsule1->length/2, 0);
                            KDL::Vector line2A = T_B_C2 * KDL::Vector(0, -capsule2->length/2, 0);
                            KDL::Vector line2B = T_B_C2 * KDL::Vector(0, capsule2->length/2, 0);
                            distanceLines(line1A, line1B, line2A, line2B, dist, p1_B, p2_B);
                        }
                        else if (geom1->type == self_collision::Geometry::CAPSULE && geom2->type == self_collision::Geometry::SPHERE) {
                            self_collision::Capsule *capsule1 = static_cast<self_collision::Capsule* >(geom1.get());
                            self_collision::Sphere *sphere2 = static_cast<self_collision::Sphere* >(geom2.get());
                            radius1 = capsule1->radius;
                            radius2 = sphere2->radius;
                            if (c_dist - radius1 - radius2 - capsule1->length/2.0 > activation_dist) {
                                continue;
                            }
                            KDL::Vector line1A = T_B_C1 * KDL::Vector(0, -capsule1->length/2, 0);
                            KDL::Vector line1B = T_B_C1 * KDL::Vector(0, capsule1->length/2, 0);
                            KDL::Vector pt2 = T_B_C2.p;
                            distanceLinePoint(line1A, line1B, pt2, dist, p1_B, p2_B);
                        }
                        else if (geom1->type == self_collision::Geometry::SPHERE && geom2->type == self_collision::Geometry::CAPSULE) {
                            self_collision::Sphere *sphere1 = static_cast<self_collision::Sphere* >(geom1.get());
                            self_collision::Capsule *capsule2 = static_cast<self_collision::Capsule* >(geom2.get());
                            radius1 = sphere1->radius;
                            radius2 = capsule2->radius;
                            if (c_dist - radius1 - radius2 - capsule2->length/2.0 > activation_dist) {
                                continue;
                            }
                            KDL::Vector pt1 = T_B_C1.p;
                            KDL::Vector line2A = T_B_C2 * KDL::Vector(0, -capsule2->length/2, 0);
                            KDL::Vector line2B = T_B_C2 * KDL::Vector(0, capsule2->length/2, 0);
                            distancePointLine(pt1, line2A, line2B, dist, p1_B, p2_B);
                        }
                        else if (geom1->type == self_collision::Geometry::SPHERE && geom2->type == self_collision::Geometry::SPHERE) {
                            self_collision::Sphere *sphere1 = static_cast<self_collision::Sphere* >(geom1.get());
                            self_collision::Sphere *sphere2 = static_cast<self_collision::Sphere* >(geom2.get());
                            radius1 = sphere1->radius;
                            radius2 = sphere2->radius;
                            if (c_dist - radius1 - radius2 > activation_dist) {
                                continue;
                            }
                            KDL::Vector pt1 = T_B_C1.p;
                            KDL::Vector pt2 = T_B_C2.p;
                            distancePoints(pt1, pt2, dist, p1_B, p2_B);
                        }
                        else {
                            continue;
                        }

                        if (dist < 100000.0) {
                            CollisionInfo col_info;
                            col_info.link1_idx = link1_idx;
                            col_info.link2_idx = link2_idx;
                            col_info.dist = dist - radius1 - radius2;
                            KDL::Vector v = p2_B - p1_B;
                            v.Normalize();
                            col_info.n1_B = v;
                            col_info.n2_B = -v;
                            col_info.p1_B = p1_B + col_info.n1_B * radius1;
                            col_info.p2_B = p2_B + col_info.n2_B * radius2;

                            if (col_info.dist < activation_dist) {
                                link_collisions.push_back(col_info);
                            }
                        }
                    }
                }
            }

            Eigen::VectorXd torque_COL(ndof);
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                torque_COL[q_idx] = 0.0;
            }
            int m_id = 1000;
            Eigen::MatrixXd N_COL(Eigen::MatrixXd::Identity(ndof, ndof));
            for (std::vector<CollisionInfo>::const_iterator it = link_collisions.begin(); it != link_collisions.end(); it++) {
                KDL::Frame &T_B_L1 = links_fk[it->link1_idx];
                const std::string &link1_name = col_model->links_[it->link1_idx]->name;
                KDL::Frame T_L1_B = T_B_L1.Inverse();
                KDL::Frame &T_B_L2 = links_fk[it->link2_idx];
                const std::string &link2_name = col_model->links_[it->link2_idx]->name;
                KDL::Frame T_L2_B = T_B_L2.Inverse();
                KDL::Vector p1_L1 = T_L1_B * it->p1_B;
                KDL::Vector p2_L2 = T_L2_B * it->p2_B;
                KDL::Vector n1_L1 = KDL::Frame(T_L1_B.M) * it->n1_B;
                KDL::Vector n2_L2 = KDL::Frame(T_L2_B.M) * it->n2_B;

                if (it->dist > 0.0) {
                    m_id = publishVectorMarker(markers_pub_, m_id, it->p1_B, it->p2_B, 1, 1, 1, 0.01, "base");
                }
                else {
                    m_id = publishVectorMarker(markers_pub_, m_id, it->p1_B, it->p2_B, 1, 0, 0, 0.01, "base");
                }

                KinematicModel::Jacobian jac1(6, ndof), jac2(6, ndof);
                kin_model.getJacobiansForPairX(jac1, jac2, link1_name, p1_L1, link2_name, p2_L2, q);

                double depth = (activation_dist - it->dist);

                // repulsive force
                double Fmax = 20.0;
                double f = 0.0;
                if (it->dist <= activation_dist) {
                    f = (it->dist - activation_dist) / activation_dist;
                }
                else {
                    f = 0.0;
                }
                double Frep = Fmax * f * f;

                double K = 2.0 * Fmax / (activation_dist * activation_dist);

                // the mapping between motions along contact normal and the Cartesian coordinates
                KDL::Vector e1 = n1_L1;
                KDL::Vector e2 = n2_L2;
                Eigen::VectorXd Jd1(3), Jd2(3);
                for (int i = 0; i < 3; i++) {
                    Jd1[i] = e1[i];
                    Jd2[i] = e2[i];
                }

                KinematicModel::Jacobian jac1_lin(3, ndof), jac2_lin(3, ndof);
                for (int q_idx = 0; q_idx < ndof; q_idx++) {
                    for (int row_idx = 0; row_idx < 3; row_idx++) {
                        jac1_lin(row_idx, q_idx) = jac1(row_idx, q_idx);
                        jac2_lin(row_idx, q_idx) = jac2(row_idx, q_idx);
                    }
                }

                KinematicModel::Jacobian Jcol1 = Jd1.transpose() * jac1_lin;
                KinematicModel::Jacobian Jcol2 = Jd2.transpose() * jac2_lin;

                KinematicModel::Jacobian Jcol(1, ndof);
                for (int q_idx = 0; q_idx < ndof; q_idx++) {
                    Jcol(0, q_idx) = Jcol1(0, q_idx) + Jcol2(0, q_idx);
                }

                // calculate relative velocity between points (1 dof)
                double ddij = (Jcol * dq)(0,0);

                double activation = 5.0*depth/activation_dist;
                if (activation > 1.0) {
                    activation = 1.0;
                }
                if (activation < 0.0) {
                    activation = 0.0;
                }
                if (ddij <= 0.0) {
                    activation = 0.0;
                }

                Eigen::MatrixXd Ncol12(ndof, ndof);
                Ncol12 = Eigen::MatrixXd::Identity(ndof, ndof) - (Jcol.transpose() * activation * Jcol);
                N_COL = N_COL * Ncol12;

                // calculate collision mass (1 dof)
                double Mdij = (Jcol * dyn_model.invI * Jcol.transpose())(0,0);

                double D = 2.0 * 0.7 * sqrt(Mdij * K);
                Eigen::VectorXd d_torque = Jcol.transpose() * (-Frep - D * ddij);
                torque_COL += d_torque;

            }
            clearMarkers(markers_pub_, m_id, 1030);

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

