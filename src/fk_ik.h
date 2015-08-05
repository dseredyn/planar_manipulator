#ifndef FK_IK_H
#define FK_IK_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include "Eigen/Dense"
#include "Eigen/LU"

class FkIkSolver {
public:
    typedef Eigen::MatrixXd Jacobian;

    FkIkSolver(const std::string &urdf_string);
    ~FkIkSolver();

    void getJacobian(Jacobian &jac, const std::string &link_name, const Eigen::VectorXd &q);
    void calculateFk(KDL::Frame &T, const std::string &link_name, const Eigen::VectorXd &q);

    KDL::Tree tree_;
protected:
    KDL::TreeJntToJacSolver *jac_solver_;
    KDL::TreeFkSolverPos_recursive *fk_solver_;
};

#endif
