#ifndef PLANAR5_DYN_MODEL_H
#define PLANAR5_DYN_MODEL_H

#include "Eigen/Dense"

class DynModelPlanar5 {
public:
    DynModelPlanar5();
    ~DynModelPlanar5();

    void coriolis(Eigen::MatrixXd &C, const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    void inertia(Eigen::MatrixXd &I, const Eigen::VectorXd &q);
    void gaussjordan(const Eigen::MatrixXd &inMatrix, Eigen::MatrixXd &outMatrix, int dim);
//    void matvecprod(double *outVector, const double *inMatrix, const double *inVector, int nRow, int nCol);
    void accel(Eigen::VectorXd &QDD, const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &t);

    Eigen::MatrixXd I, invI, C;
    Eigen::VectorXd tmpTau;
};

#endif
