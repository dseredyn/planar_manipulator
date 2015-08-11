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

#ifndef TASK_JLC_H__
#define TASK_JLC_H__

#include "Eigen/Dense"
#include "Eigen/LU"

class Task_JLC {
public:
    Task_JLC(const Eigen::VectorXd &lower_limit, const Eigen::VectorXd &upper_limit, const Eigen::VectorXd &limit_range, const Eigen::VectorXd &max_trq);

    ~Task_JLC();

    double jointLimitTrq(double hl, double ll, double ls, double r_max, double q, double &out_limit_activation);

    void compute(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &I, Eigen::VectorXd &torque, Eigen::MatrixXd &N);

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

#endif  // TASK_JLC_H__

