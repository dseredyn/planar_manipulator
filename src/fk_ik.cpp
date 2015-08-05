#include "fk_ik.h"

//#include <stdlib.h>
//#include <math.h>


FkIkSolver::FkIkSolver(const std::string &urdf_string) :
    jac_solver_(NULL)
{
    if (!kdl_parser::treeFromString(urdf_string, tree_)){
        std::cout << "Failed to construct kdl tree" << std::endl;
        return;
    }

    jac_solver_ = new KDL::TreeJntToJacSolver(tree_);
    fk_solver_ = new KDL::TreeFkSolverPos_recursive(tree_);
}

FkIkSolver::~FkIkSolver() {
    if (jac_solver_ != NULL) {
        delete jac_solver_;
        jac_solver_ = NULL;
    }

    if (fk_solver_ != NULL) {
        delete fk_solver_;
        fk_solver_ = NULL;
    }
}

void FkIkSolver::getJacobian(Jacobian &jac, const std::string &link_name, const Eigen::VectorXd &q) {
    KDL::JntArray q_in( q.innerSize() );
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        q_in(q_idx) = q[q_idx];
    }
    KDL::Jacobian jac_out( q.innerSize() );
    SetToZero(jac_out);
    jac_solver_->JntToJac(q_in, jac_out, link_name);
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        KDL::Twist t = jac_out.getColumn(q_idx);
        for (int d_idx = 0; d_idx < 6; d_idx++) {
            jac(d_idx, q_idx) = t[d_idx];
        }
    }
}

void FkIkSolver::calculateFk(KDL::Frame &T, const std::string &link_name, const Eigen::VectorXd &q) {
    KDL::JntArray q_in( q.innerSize() );
    for (int q_idx = 0; q_idx < q.innerSize(); q_idx++) {
        q_in(q_idx) = q[q_idx];
    }
    fk_solver_->JntToCart(q_in, T, link_name);
}

