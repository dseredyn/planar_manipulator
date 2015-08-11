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

#include "task_jlc.h"

    Task_JLC::Task_JLC(const Eigen::VectorXd &lower_limit, const Eigen::VectorXd &upper_limit, const Eigen::VectorXd &limit_range, const Eigen::VectorXd &max_trq) :
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

    Task_JLC::~Task_JLC() {
    }

    double Task_JLC::jointLimitTrq(double hl, double ll, double ls,
        double r_max, double q, double &out_limit_activation) {
        if (q > hl) {
            q = hl;
        }
        else if (q < ll) {
            q = ll;
        }
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

    void Task_JLC::compute(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &I, Eigen::VectorXd &torque, Eigen::MatrixXd &N) {
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

