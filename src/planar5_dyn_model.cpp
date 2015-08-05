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

#include "planar5_dyn_model.h"

#include <stdlib.h>
#include <math.h>

DynModelPlanar5::DynModelPlanar5() {
    I.resize(5,5);
    invI.resize(5,5);
    C.resize(5,5);
    tmpTau.resize(5);
}

DynModelPlanar5::~DynModelPlanar5() {
}
 
void DynModelPlanar5::coriolis(Eigen::MatrixXd &C, const Eigen::VectorXd &q, const Eigen::VectorXd &dq){
    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];
    double q4 = q[3];
    double q5 = q[4];
    double qd1 = dq[0];
    double qd2 = dq[1];
    double qd3 = dq[2];
    double qd4 = dq[3];
    double qd5 = dq[4];
 
	/* call the row routines */
  C(0,0) = qd2*(sin(q2+q3+q4)*(2.0/5.0)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q2)*(1.8E1/2.5E1))*(-1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0);
  C(1,0) = -qd2*(sin(q2+q3+q4)*(1.0/5.0)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3)*(7.0/2.5E1)+sin(q2)*(9.0/2.5E1))-qd1*(sin(q2+q3+q4)*(2.0/5.0)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q2)*(1.8E1/2.5E1))*(1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0);
  C(2,0) = qd1*sin(q2+q3+q4+q5)*(-2.0/2.5E1)-qd2*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd1*sin(q2+q3)*(7.0/2.5E1)-qd2*sin(q2+q3)*(7.0/2.5E1)-qd1*sin(q3+q4)*(1.0/5.0)-qd3*sin(q2+q3)*(7.0/2.5E1)-qd2*sin(q3+q4)*(1.0/5.0)-qd3*sin(q3+q4)*(1.0/5.0)-qd4*sin(q3+q4)*(1.0/5.0)-qd4*sin(q4+q5)*(2.0/2.5E1)-qd5*sin(q4+q5)*(2.0/2.5E1)-qd1*sin(q3)*(7.0/2.5E1)-qd2*sin(q3)*(7.0/2.5E1)-qd3*sin(q3)*(7.0/2.5E1)-qd4*sin(q4)*(1.0/5.0)-qd5*sin(q5)*(2.0/2.5E1)-qd1*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q2+q3+q4)*(1.0/5.0)-qd3*sin(q2+q3+q4)*(1.0/5.0)-qd1*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q3+q4+q5)*(2.0/2.5E1);
  C(3,0) = qd1*sin(q2+q3+q4+q5)*(-2.0/2.5E1)-qd2*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd1*sin(q3+q4)*(1.0/5.0)-qd2*sin(q3+q4)*(1.0/5.0)-qd1*sin(q4+q5)*(2.0/2.5E1)-qd3*sin(q3+q4)*(1.0/5.0)-qd2*sin(q4+q5)*(2.0/2.5E1)-qd4*sin(q3+q4)*(1.0/5.0)-qd3*sin(q4+q5)*(2.0/2.5E1)-qd4*sin(q4+q5)*(2.0/2.5E1)-qd5*sin(q4+q5)*(2.0/2.5E1)-qd1*sin(q4)*(1.0/5.0)-qd2*sin(q4)*(1.0/5.0)-qd3*sin(q4)*(1.0/5.0)-qd4*sin(q4)*(1.0/5.0)-qd5*sin(q5)*(2.0/2.5E1)-qd1*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q2+q3+q4)*(1.0/5.0)-qd3*sin(q2+q3+q4)*(1.0/5.0)-qd1*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q3+q4+q5)*(2.0/2.5E1);
  C(4,0) = (sin(q3+q4+q5)+sin(q2+q3+q4+q5)+sin(q4+q5)+sin(q5))*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

  C(0,1) = qd1*(sin(q2+q3+q4)*(1.0/5.0)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3)*(7.0/2.5E1)+sin(q2)*(9.0/2.5E1))-qd3*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
  C(1,1) = qd3*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(-1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
  C(2,1) = -qd3*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q3)*(7.0/2.5E1))-qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
  C(3,1) = qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(-1.0/2.0)-qd4*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))-qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
  C(4,1) = (sin(q3+q4+q5)+sin(q4+q5)+sin(q5))*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

  C(0,2) = qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd4*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd1*(sin(q2+q3+q4)*(1.0/5.0)+sin(q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3)*(7.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q3)*(7.0/2.5E1));
  C(1,2) = qd2*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q3)*(7.0/2.5E1))+qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd4*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0);
  C(2,2) = qd4*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(-1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0);
  C(3,2) = qd1*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(-1.0/2.0)-qd4*(sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))-qd2*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0);
  C(4,2) = (sin(q4+q5)+sin(q5))*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

  C(0,3) = qd1*sin(q2+q3+q4+q5)*(2.0/2.5E1)+qd1*sin(q3+q4)*(1.0/5.0)+qd2*sin(q3+q4)*(1.0/5.0)+qd1*sin(q4+q5)*(2.0/2.5E1)+qd2*sin(q4+q5)*(2.0/2.5E1)+qd3*sin(q4+q5)*(2.0/2.5E1)+qd1*sin(q4)*(1.0/5.0)+qd2*sin(q4)*(1.0/5.0)+qd3*sin(q4)*(1.0/5.0)-qd5*sin(q5)*(2.0/2.5E1)+qd1*sin(q2+q3+q4)*(1.0/5.0)+qd1*sin(q3+q4+q5)*(2.0/2.5E1)+qd2*sin(q3+q4+q5)*(2.0/2.5E1);
  C(1,3) = qd2*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))+qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)+qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*sin(q5)*(2.0/2.5E1);
  C(2,3) = qd3*(sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))+qd1*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)+qd2*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*sin(q5)*(2.0/2.5E1);
  C(3,3) = qd5*sin(q5)*(-2.0/2.5E1);
  C(4,3) = sin(q5)*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

  C(0,4) = qd1*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q4+q5)*(2.0/2.5E1)+sin(q5)*(2.0/2.5E1))+qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd4*sin(q5)*(2.0/2.5E1);
  C(1,4) = qd2*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q4+q5)*(2.0/2.5E1)+sin(q5)*(2.0/2.5E1))+qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd4*sin(q5)*(2.0/2.5E1);
  C(2,4) = qd3*(sin(q4+q5)*(2.0/2.5E1)+sin(q5)*(2.0/2.5E1))+qd1*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd2*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd4*sin(q5)*(2.0/2.5E1);
  C(3,4) = sin(q5)*(qd1+qd2+qd3+qd4)*(2.0/2.5E1);
  C(4,4) = 0.0;
}


void DynModelPlanar5::inertia(Eigen::MatrixXd &I, const Eigen::VectorXd &q){
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];
 
  I(0,0) = cos(q2+q3+q4)*(2.0/5.0)+cos(q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3)*(1.4E1/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q2)*(1.8E1/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.3E1/5.0;
  I(1,0) = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q2)*(9.0/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.9E1/1.0E1;
  I(2,0) = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
  I(3,0) = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(4,0) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;

  I(0,1) = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q2)*(9.0/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.9E1/1.0E1;
  I(1,1) = cos(q3+q4+q5)*(4.0/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.9E1/1.0E1;
  I(2,1) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
  I(3,1) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(4,1) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;

  I(0,2) = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
  I(1,2) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
  I(2,2) = cos(q4+q5)*(4.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
  I(3,2) = cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(4,2) = cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;

  I(0,3) = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(1,3) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(2,3) = cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(3,3) = cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
  I(4,3) = cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
 
  I(0,4) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
  I(1,4) = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
  I(2,4) = cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
  I(3,4) = cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
  I(4,4) = 7.0/2.5E1;
}

void DynModelPlanar5::gaussjordan(const Eigen::MatrixXd &inMatrix, Eigen::MatrixXd &outMatrix, int dim){
 
	int iRow, iCol, diagIndex;
	double diagFactor, tmpFactor;
    Eigen::MatrixXd inMatrixCopy(inMatrix);
//	double* inMatrixCopy = (double*) malloc(dim*dim*sizeof(double));
 
	/* make deep copy of input matrix */
	for(iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy(iCol,iRow) = inMatrix(iCol,iRow);
		}
	}
	/* Initialize output matrix as identity matrix. */
	for (iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++ ){
			if (iCol == iRow){
				outMatrix(iCol,iRow) = 1;
			}
			else{
				outMatrix(iCol,iRow) = 0;
			}
		}
	}
 
	for (diagIndex = 0; diagIndex < dim; diagIndex++ )
	{
		/* determine diagonal factor */
		diagFactor = inMatrixCopy(diagIndex,diagIndex);
 
		/* divide column entries by diagonal factor */
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy(iCol,diagIndex) /= diagFactor;
			outMatrix(iCol,diagIndex) /= diagFactor;
		}
 
		/* perform line-by-line elimination */
		for (iRow = 0; iRow < dim; iRow++){
			if (iRow != diagIndex){
				tmpFactor = inMatrixCopy(diagIndex,iRow);
 
				for(iCol = 0; iCol < dim; iCol++){
				inMatrixCopy(iCol,iRow)  -= inMatrixCopy(iCol,diagIndex)*tmpFactor;
				outMatrix(iCol,iRow) -= outMatrix(iCol,diagIndex)*tmpFactor;
				}
			}
		} /* line-by-line elimination */
 
	}
//ree(inMatrixCopy);
}

/*void DynModelPlanar5::matvecprod(double *outVector, const double *inMatrix, const double *inVector, int nRow, int nCol){
	int iRow, iCol = 0;
 
	for (iRow = 0; iRow < nRow; iRow++){
		for (iCol = 0; iCol < nCol; iCol++){
			outVector[iCol] += inMatrix[nRow*iRow+iCol] * inVector[iRow];
		}
	}  
}*/

void DynModelPlanar5::computeM(const Eigen::VectorXd &q) {
	inertia(I, q);
	gaussjordan(I, invI, 5);
}

void DynModelPlanar5::accel(Eigen::VectorXd &QDD, const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &t){

	/* declare variables */
	int iCol;
//	double I[5][5];
//	double invI[5][5];
//	double C[5][5];
//	double gravload[5][1];
//	double friction[5][1];
//	double tmpTau[1][5];
 
	/* call the computational routines */
	coriolis(C, q, dq);
//	gravload(gravload, q);
//	friction(friction, dq);
 
	/* fill temporary vector */
    tmpTau = C * dq;
//	matvecprod(&tmpTau[0][0], &C[0][0], dq,5,5);
	for (iCol = 0; iCol < 5; iCol++){
//		tmpTau[0][iCol] = t[iCol] -  tmpTau[0][iCol] - gravload[iCol][0] + friction[iCol][0];
		tmpTau[iCol] = t[iCol] -  tmpTau[iCol] - dq[iCol] * 0.5;
	}
	/* compute acceleration */
    QDD = invI * tmpTau;
//	matvecprod(&QDD[0][0], &invI[0][0], &tmpTau[0][0],5,5);
}

