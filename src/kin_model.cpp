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

#include "kin_model.h"

#include <set>

KinematicModel::KinematicModel(const std::string &urdf_string, const std::vector<std::string > &joint_names) :
    jac_solver_(NULL),
    fk_solver_(NULL)
{
    if (!kdl_parser::treeFromString(urdf_string, tree_)){
        std::cout << "Failed to construct kdl tree" << std::endl;
        return;
    }

    jac_solver_ = new KDL::TreeJntToJacSolver(tree_);
    fk_solver_ = new KDL::TreeFkSolverPos_recursive(tree_);

    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegments().begin(); seg_it != tree_.getSegments().end(); seg_it++) {
        for (int q_idx = 0; q_idx < joint_names.size(); q_idx++) {
            if (joint_names[q_idx] == seg_it->second.segment.getJoint().getName()) {
                q_idx_q_nr_map_.insert( std::make_pair(q_idx, seg_it->second.q_nr) );
                q_nr_q_idx_map_.insert( std::make_pair(seg_it->second.q_nr, q_idx) );
            }
        }
    }
}

KinematicModel::~KinematicModel() {
    if (jac_solver_ != NULL) {
        delete jac_solver_;
        jac_solver_ = NULL;
    }

    if (fk_solver_ != NULL) {
        delete fk_solver_;
        fk_solver_ = NULL;
    }
}

void KinematicModel::getJacobian(Jacobian &jac, const std::string &link_name, const Eigen::VectorXd &q) {
    KDL::JntArray q_in( q.innerSize() );
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        int q_nr = q_idx_q_nr_map_[q_idx];
        q_in(q_nr) = q[q_idx];
    }

    KDL::Jacobian jac_out( q.innerSize() );
    SetToZero(jac_out);
    jac_solver_->JntToJac(q_in, jac_out, link_name);
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        int q_nr = q_idx_q_nr_map_[q_idx];
        KDL::Twist t = jac_out.getColumn(q_nr);
        for (int d_idx = 0; d_idx < 6; d_idx++) {
            jac(d_idx, q_idx) = t[d_idx];
        }
    }
}

void KinematicModel::calculateFk(KDL::Frame &T, const std::string &link_name, const Eigen::VectorXd &q) {
    KDL::JntArray q_in( q.innerSize() );
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        int q_nr = q_idx_q_nr_map_[q_idx];
        q_in(q_nr) = q[q_idx];
    }
    fk_solver_->JntToCart(q_in, T, link_name);
}

void KinematicModel::getJacobiansForPairX(Jacobian &jac1, Jacobian &jac2,
                                        const std::string &link_name1, const KDL::Vector &x1,
                                        const std::string &link_name2, const KDL::Vector &x2, const Eigen::VectorXd &q) {

    std::set<unsigned int> link1_chain;
    KDL::SegmentMap::const_iterator root = tree_.getRootSegment();
    KDL::SegmentMap::const_iterator end = tree_.getSegments().end();

    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegment(link_name1); seg_it != root && seg_it != end; seg_it = seg_it->second.parent) {
        if (seg_it->second.segment.getJoint().getType() == KDL::Joint::None) {
            continue;
        }
        link1_chain.insert(seg_it->second.q_nr);
    }

    std::string common_link_name;

    for (KDL::SegmentMap::const_iterator seg_it = tree_.getSegment(link_name2); seg_it != root && seg_it != end; seg_it = seg_it->second.parent) {
        if (seg_it->second.segment.getJoint().getType() == KDL::Joint::None) {
            continue;
        }
        if (link1_chain.count(seg_it->second.q_nr) > 0) {
            common_link_name = seg_it->second.segment.getName();
            break;
        }
    }

    getJacobianForX(jac1, link_name1, x1, q, common_link_name);
    getJacobianForX(jac2, link_name2, x2, q, common_link_name);
}

void KinematicModel::getJacobianForX(Jacobian &jac, const std::string &link_name, const KDL::Vector &x, const Eigen::VectorXd &q, const std::string &base_name) {
        // Lets search the tree-element
        // If segmentname is not inside the tree, back out:
        // Let's make the jacobian zero:
        for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
            for (int dof_idx = 0; dof_idx < 6; dof_idx++) {
                jac(dof_idx, q_idx) = 0.0;
            }
        }

        KDL::Frame T_total(x);
        KDL::SegmentMap::const_iterator root;
        if (base_name.empty()) {
            root = tree_.getRootSegment();
        }
        else {
            root = tree_.getSegment(base_name);
        }

        // Lets recursively iterate until we are in the root segment
        KDL::SegmentMap::const_iterator it = tree_.getSegment(link_name);
        if (it == tree_.getSegments().end()) {
            // link_name not found in the kinematic model - return zero jacobian
            return;
        }
        while (it != root) {    //l_index != root_index:
            // get the corresponding q_nr for this TreeElement:
            // get the pose of the segment:

            const KDL::Segment &seg_kdl = it->second.segment;
            int q_idx = -1;
            double q_seg = 0.0;
            if (seg_kdl.getJoint().getType() == KDL::Joint::None) {
                q_idx = -1;
                q_seg = 0.0;
            }
            else {
                q_idx = q_nr_q_idx_map_[it->second.q_nr];
                q_seg = q[q_idx];
            }

            KDL::Frame T_local = seg_kdl.pose(q_seg);
            // calculate new T_end:
            T_total = T_local * T_total;
            // get the twist of the segment:
            KDL::Twist t_local = seg_kdl.twist(q_seg, 1.0);
            // transform the endpoint of the local twist to the global endpoint:
            t_local = t_local.RefPoint(T_total.p - T_local.p);
            // transform the base of the twist to the endpoint
            t_local = T_total.M.Inverse(t_local);
            // store the twist in the jacobian:
            if (q_idx != -1) {
                for (int dof_idx = 0; dof_idx < 6; dof_idx++) {
                    jac(dof_idx, q_idx) = t_local[dof_idx];
                }
            }

            // goto the parent
            it = it->second.parent;
        }

        // Change the base of the complete jacobian from the endpoint to the base
        // NOT!
//        changeBase(jac, T_total.M, jac);
//        jac.changeBase(T_total.M)
//        return 0;
}

