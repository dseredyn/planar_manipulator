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

#include "task_hand.h"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#define RESTRICT_ALLOC
#define UNRESTRICT_ALLOC
#endif

    Task_HAND::Task_HAND(int ndof, int dim) :
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
        tmpKN_(dim_, ndof_),
        Ji_(ndof_, dim_),
        wrench_tmp(dim_)
    {
        lu_ = Eigen::PartialPivLU<Eigen::MatrixXd>(ndof_);
        luKK_ = Eigen::PartialPivLU<Eigen::MatrixXd>(dim_);
    }

    Task_HAND::~Task_HAND() {
    }

    void Task_HAND::compute(const Eigen::VectorXd &T_diff, const Eigen::VectorXd &Kc, const Eigen::VectorXd &Dxi,
                            const Eigen::MatrixXd &J, const Eigen::VectorXd &dq, const Eigen::MatrixXd &invI,
                            Eigen::VectorXd &torque, Eigen::MatrixXd &N)
    {
            for (int dim_idx = 0; dim_idx < dim_; dim_idx++) {
                wrench_[dim_idx] = Kc[dim_idx] * T_diff[dim_idx];
            }
            torque = J.transpose() * wrench_;

            // code form cartesian_impedance.h
            JT = J.transpose();
//            lu_.compute(invI);
//            Eigen::MatrixXd Mi = lu_.inverse();
            Eigen::MatrixXd Mi = invI;

            tmpNK_.noalias() = J * Mi;
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
            wrench_tmp.noalias() = Dc * tmpK_;
            torque.noalias() -= JT * wrench_tmp;



    tmpNK_.noalias() = J * Mi;
    tmpKK_.noalias() = tmpNK_ * JT;
    luKK_.compute(tmpKK_);
    tmpKK_ = luKK_.inverse();
    tmpKN_.noalias() = Mi * JT;
    Ji_.noalias() = tmpKN_ * tmpKK_;

    N.noalias() = Eigen::MatrixXd::Identity(N.rows(), N.cols());
    N.noalias() -=  JT * A * J * Mi;

//            N = Eigen::MatrixXd::Identity(ndof_, ndof_) - (JT * J);
//            N = Eigen::MatrixXd::Identity(ndof_, ndof_) - (Ji * J);
    }

