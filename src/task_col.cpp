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

#include "task_col.h"

Task_COL::Task_COL(int ndof, double activation_dist, double Fmax, const KinematicModel &kin_model, const boost::shared_ptr<self_collision::CollisionModel> &col_model) :
        ndof_(ndof),
        activation_dist_(activation_dist),
        Fmax_(Fmax),
        kin_model_(kin_model)
    {
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            link_names_vec_.push_back(col_model->getLinkName(l_idx));
        }
    }

    Task_COL::~Task_COL() {
    }

void Task_COL::compute(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::MatrixXd &invI, const std::vector<KDL::Frame > &links_fk, const std::vector<CollisionInfo> &link_collisions, Eigen::VectorXd &torque_COL, Eigen::MatrixXd &N_COL) {
            for (std::vector<CollisionInfo>::const_iterator it = link_collisions.begin(); it != link_collisions.end(); it++) {
                const KDL::Frame &T_B_L1 = links_fk[it->link1_idx];
                const std::string &link1_name = link_names_vec_[it->link1_idx];
                const std::string &link2_name = link_names_vec_[it->link2_idx];

                KDL::Frame T_L1_B = T_B_L1.Inverse();
                const KDL::Frame &T_B_L2 = links_fk[it->link2_idx];
                KDL::Frame T_L2_B = T_B_L2.Inverse();
                KDL::Vector p1_L1 = T_L1_B * it->p1_B;
                KDL::Vector p2_L2 = T_L2_B * it->p2_B;
                KDL::Vector n1_L1 = KDL::Frame(T_L1_B.M) * it->n1_B;
                KDL::Vector n2_L2 = KDL::Frame(T_L2_B.M) * it->n2_B;

                // visualization
/*                if (it->dist > 0.0) {
                    m_id = publishVectorMarker(markers_pub_, m_id, it->p1_B, it->p2_B, 1, 1, 1, 0.01, "base");
                }
                else {
                    m_id = publishVectorMarker(markers_pub_, m_id, it->p1_B, it->p2_B, 1, 0, 0, 0.01, "base");
                }
*/
                KinematicModel::Jacobian jac1(6, ndof_), jac2(6, ndof_);
                kin_model_.getJacobiansForPairX(jac1, jac2, link1_name, p1_L1, link2_name, p2_L2, q);

                double depth = (activation_dist_ - it->dist);

                // repulsive force
                double f = 0.0;
                if (it->dist <= activation_dist_) {
                    f = (activation_dist_ - it->dist) / activation_dist_;
                }
                else {
                    f = 0.0;
                }

                if (f > 1.0) {
                    f = 1.0;
                }
                double Frep = Fmax_ * f * f;

                double K = 2.0 * Fmax_ / (activation_dist_ * activation_dist_);

                // the mapping between motions along contact normal and the Cartesian coordinates
                KDL::Vector e1 = n1_L1;
                KDL::Vector e2 = n2_L2;
                Eigen::VectorXd Jd1(3), Jd2(3);
                for (int i = 0; i < 3; i++) {
                    Jd1[i] = e1[i];
                    Jd2[i] = e2[i];
                }

                KinematicModel::Jacobian jac1_lin(3, ndof_), jac2_lin(3, ndof_);
                for (int q_idx = 0; q_idx < ndof_; q_idx++) {
                    for (int row_idx = 0; row_idx < 3; row_idx++) {
                        jac1_lin(row_idx, q_idx) = jac1(row_idx, q_idx);
                        jac2_lin(row_idx, q_idx) = jac2(row_idx, q_idx);
                    }
                }

                KinematicModel::Jacobian Jcol1 = Jd1.transpose() * jac1_lin;
                KinematicModel::Jacobian Jcol2 = Jd2.transpose() * jac2_lin;

                KinematicModel::Jacobian Jcol(1, ndof_);
                for (int q_idx = 0; q_idx < ndof_; q_idx++) {
                    Jcol(0, q_idx) = Jcol1(0, q_idx) + Jcol2(0, q_idx);
                }

                // calculate relative velocity between points (1 dof)
                double ddij = (Jcol * dq)(0,0);

                double activation = 5.0*depth/activation_dist_;
                if (activation > 1.0) {
                    activation = 1.0;
                }
                if (activation < 0.0) {
                    activation = 0.0;
                }
//                if (ddij <= 0.0) {
//                    activation = 0.0;
//                }

                Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jcol, Eigen::ComputeFullV);

                Eigen::MatrixXd activation_matrix = Eigen::MatrixXd::Zero(ndof_, ndof_);
                activation_matrix(0,0) = activation;
                activation_matrix(1,1) = activation;

                Eigen::MatrixXd Ncol12(ndof_, ndof_);
//                Ncol12 = Eigen::MatrixXd::Identity(ndof_, ndof_) - (Jcol.transpose() * activation * Jcol);
                Ncol12 = Eigen::MatrixXd::Identity(ndof_, ndof_) - (svd.matrixV() * activation_matrix * svd.matrixV().transpose());
                N_COL = N_COL * Ncol12;

                // calculate collision mass (1 dof)
                double Mdij = (Jcol * invI * Jcol.transpose())(0,0);

                double D = 2.0 * 0.7 * sqrt(Mdij * K);
                Eigen::VectorXd d_torque = Jcol.transpose() * (-Frep - D * ddij);
                torque_COL += d_torque;
            }
    }

