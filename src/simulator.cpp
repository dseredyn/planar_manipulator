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

#include "simulator.h"

//#include <boost/bind.hpp>
//#include <string>
//#include <stdlib.h>
//#include <stdio.h>

//#include "Eigen/Dense"

//#include "planar5_dyn_model.h"
//#include <collision_convex_model/collision_convex_model.h>
//#include "kin_model/kin_model.h"
//#include "marker_publisher.h"
//#include "task_col.h"
//#include "task_hand.h"
//#include "task_jlc.h"
//#include "random_uniform.h"

    DynamicsSimulatorHandPose::DynamicsSimulatorHandPose(int ndof, const std::string &effector_name, const boost::shared_ptr<self_collision::CollisionModel> &col_model,
                        const boost::shared_ptr<KinematicModel> &kin_model,
                        const boost::shared_ptr<DynamicModel > &dyn_model,
                        const std::vector<std::string > &joint_names) :
        ndof_(ndof),
        effector_name_(effector_name),
        q_(ndof_),
        dq_(ndof_),
        ddq_(ndof_),
        torque_(ndof_),
        col_model_(col_model),
        kin_model_(kin_model),
        dyn_model_(dyn_model),
        links_fk_(col_model->getLinksCount()),
        activation_dist_(0.05)
    {
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q_[q_idx] = 0.0;
            dq_[q_idx] = 0.0;
            ddq_[q_idx] = 0.0;
            torque_[q_idx] = 0.0;
        }

        effector_idx_ = col_model->getLinkIndex(effector_name_);

        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof), limit_range(ndof), max_trq(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {

            if (!col_model->getJointLimits( (*name_it), lower_limit[q_idx], upper_limit[q_idx] )) {
                std::cout << "ERROR: could not find joint with name " << (*name_it) << std::endl;
                return;
            }
            limit_range[q_idx] = 10.0 / 180.0 * 3.141592653589793;
            max_trq[q_idx] = 10.0;
        }

        task_JLC_.reset( new Task_JLC(lower_limit, upper_limit, limit_range, max_trq) );
        task_COL_.reset( new Task_COL(ndof_, activation_dist_, 10.0, kin_model_, col_model_) );
        task_HAND_.reset( new Task_HAND(ndof_, 3) );

        J_r_HAND_6_.resize(6, ndof_);
        J_r_HAND_.resize(3, ndof_);
    }

    void DynamicsSimulatorHandPose::setState(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &ddq) {
        q_ = q;
        dq_ = dq;
        ddq_ = ddq;
    }

    void DynamicsSimulatorHandPose::getState(Eigen::VectorXd &q, Eigen::VectorXd &dq, Eigen::VectorXd &ddq) {
        q = q_;
        dq = dq_;
        ddq = ddq_;
    }

    void DynamicsSimulatorHandPose::setTarget(const KDL::Frame &r_HAND_target) {
        r_HAND_target_ = r_HAND_target;
    }

    void DynamicsSimulatorHandPose::oneStep(const KDL::Twist &diff) {
                // calculate forward kinematics for all links
                for (int l_idx = 0; l_idx < col_model_->getLinksCount(); l_idx++) {
                    kin_model_->calculateFk(links_fk_[l_idx], col_model_->getLinkName(l_idx), q_);
                }
                // calculate inertia matrix for whole body
                dyn_model_->computeM(q_);

                //
                // joint limit avoidance
                //
                Eigen::VectorXd torque_JLC(ndof_);
                Eigen::MatrixXd N_JLC(ndof_, ndof_);
                task_JLC_->compute(q_, dq_, dyn_model_->getM(), torque_JLC, N_JLC);

                //
                // collision constraints
                //
                std::vector<self_collision::CollisionInfo> link_collisions;
                self_collision::getCollisionPairs(col_model_, links_fk_, activation_dist_, link_collisions);

                Eigen::VectorXd torque_COL(ndof_);
                for (int q_idx = 0; q_idx < ndof_; q_idx++) {
                    torque_COL[q_idx] = 0.0;
                }
                Eigen::MatrixXd N_COL(Eigen::MatrixXd::Identity(ndof_, ndof_));

                task_COL_->compute(q_, dq_, dyn_model_->getInvM(), links_fk_, link_collisions, torque_COL, N_COL);

                //
                // effector task
                //

                Eigen::VectorXd torque_HAND(ndof_);
                Eigen::MatrixXd N_HAND(ndof_, ndof_);

                Eigen::VectorXd r_HAND_diff(3);
                r_HAND_diff[0] = diff[0];
                r_HAND_diff[1] = diff[1];
                r_HAND_diff[2] = diff[5];

                Eigen::VectorXd Kc(3);
                Kc[0] = 10.0;
                Kc[1] = 10.0;
                Kc[2] = 1.00;
                Eigen::VectorXd Dxi(3);
                Dxi[0] = 0.7;
                Dxi[1] = 0.7;
                Dxi[2] = 0.7;

                kin_model_->getJacobian(J_r_HAND_6_, effector_name_, q_);

                for (int q_idx = 0; q_idx < ndof_; q_idx++) {
                    J_r_HAND_(0, q_idx) = J_r_HAND_6_(0, q_idx);
                    J_r_HAND_(1, q_idx) = J_r_HAND_6_(1, q_idx);
                    J_r_HAND_(2, q_idx) = J_r_HAND_6_(5, q_idx);
                }

                task_HAND_->compute(r_HAND_diff, Kc, Dxi, J_r_HAND_, dq_, dyn_model_->getInvM(), torque_HAND, N_HAND);

                torque_ = torque_JLC + N_JLC.transpose() * (torque_COL + (N_COL.transpose() * (torque_HAND)));


                // simulate one step
                Eigen::VectorXd prev_ddq(ddq_), prev_dq(dq_);
                dyn_model_->accel(ddq_, q_, dq_, torque_);
                float time_d = 0.01;
                for (int q_idx = 0; q_idx < ndof_; q_idx++) {
                    dq_[q_idx] += (prev_ddq[q_idx] + ddq_[q_idx]) / 2.0 * time_d;
                    q_[q_idx] += (prev_dq[q_idx] + dq_[q_idx]) / 2.0 * time_d;
                }
    }

    void DynamicsSimulatorHandPose::oneStep() {
        KDL::Frame r_HAND_current;
        kin_model_->calculateFk(r_HAND_current, effector_name_, q_);
        KDL::Twist diff = KDL::diff(r_HAND_current, r_HAND_target_, 1.0);
        oneStep(diff);
    }

