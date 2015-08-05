# Copyright (c) 2015, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import PyKDL
from math import sin, cos
import numpy as np
import copy

class DynModelPlanar5:

    def __init__(self):
        pass

    # void Planar5DOF_inertia(double I[][5], const double* input1){
    def inertia(self, q):
        q2 = q[1,0]
        q3 = q[2,0]
        q4 = q[3,0]
        q5 = q[4,0]
        I = np.matrix(np.zeros( (5,5) ))
        # call the row routines
        I[0,0] = cos(q2+q3+q4)*(2.0/5.0)+cos(q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3)*(1.4E1/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q2)*(1.8E1/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.3E1/5.0;
        I[1,0] = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q2)*(9.0/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.9E1/1.0E1;
        I[2,0] = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
        I[3,0] = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[4,0] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;

        I[0,1] = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(4.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q2)*(9.0/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.9E1/1.0E1;
        I[1,1] = cos(q3+q4+q5)*(4.0/2.5E1)+cos(q3+q4)*(2.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(1.4E1/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+1.9E1/1.0E1;
        I[2,1] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
        I[3,1] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[4,1] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;

        I[0,2] = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3)*(7.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
        I[1,2] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(4.0/2.5E1)+cos(q3)*(7.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
        I[2,2] = cos(q4+q5)*(4.0/2.5E1)+cos(q4)*(2.0/5.0)+cos(q5)*(4.0/2.5E1)+3.2E1/2.5E1;
        I[3,2] = cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[4,2] = cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;

        I[0,3] = cos(q2+q3+q4)*(1.0/5.0)+cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[1,3] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q3+q4)*(1.0/5.0)+cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[2,3] = cos(q4+q5)*(2.0/2.5E1)+cos(q4)*(1.0/5.0)+cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[3,3] = cos(q5)*(4.0/2.5E1)+3.7E1/5.0E1;
        I[4,3] = cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
 
        I[0,4] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q2+q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
        I[1,4] = cos(q3+q4+q5)*(2.0/2.5E1)+cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
        I[2,4] = cos(q4+q5)*(2.0/2.5E1)+cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
        I[3,4] = cos(q5)*(2.0/2.5E1)+7.0/2.5E1;
        I[4,4] = 7.0/2.5E1;

        return I

    #void Planar5DOF_coriolis(double C[][5], const double* input1, const double* input2){
    def coriolis(self, q, dq):
        q1 = q[0,0];
        q2 = q[1,0];
        q3 = q[2,0];
        q4 = q[3,0];
        q5 = q[4,0];
        qd1 = dq[0,0];
        qd2 = dq[1,0];
        qd3 = dq[2,0];
        qd4 = dq[3,0];
        qd5 = dq[4,0];
        C = np.matrix(np.zeros( (5,5) ))

        C[0,0] = qd2*(sin(q2+q3+q4)*(2.0/5.0)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q2)*(1.8E1/2.5E1))*(-1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0);
        C[1,0] = -qd2*(sin(q2+q3+q4)*(1.0/5.0)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3)*(7.0/2.5E1)+sin(q2)*(9.0/2.5E1))-qd1*(sin(q2+q3+q4)*(2.0/5.0)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q2)*(1.8E1/2.5E1))*(1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q2+q3+q4)*(2.0/5.0)+sin(q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3+q4+q5)*(4.0/2.5E1)+sin(q2+q3)*(1.4E1/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0);
        C[2,0] = qd1*sin(q2+q3+q4+q5)*(-2.0/2.5E1)-qd2*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd1*sin(q2+q3)*(7.0/2.5E1)-qd2*sin(q2+q3)*(7.0/2.5E1)-qd1*sin(q3+q4)*(1.0/5.0)-qd3*sin(q2+q3)*(7.0/2.5E1)-qd2*sin(q3+q4)*(1.0/5.0)-qd3*sin(q3+q4)*(1.0/5.0)-qd4*sin(q3+q4)*(1.0/5.0)-qd4*sin(q4+q5)*(2.0/2.5E1)-qd5*sin(q4+q5)*(2.0/2.5E1)-qd1*sin(q3)*(7.0/2.5E1)-qd2*sin(q3)*(7.0/2.5E1)-qd3*sin(q3)*(7.0/2.5E1)-qd4*sin(q4)*(1.0/5.0)-qd5*sin(q5)*(2.0/2.5E1)-qd1*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q2+q3+q4)*(1.0/5.0)-qd3*sin(q2+q3+q4)*(1.0/5.0)-qd1*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q3+q4+q5)*(2.0/2.5E1);
        C[3,0] = qd1*sin(q2+q3+q4+q5)*(-2.0/2.5E1)-qd2*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q2+q3+q4+q5)*(2.0/2.5E1)-qd1*sin(q3+q4)*(1.0/5.0)-qd2*sin(q3+q4)*(1.0/5.0)-qd1*sin(q4+q5)*(2.0/2.5E1)-qd3*sin(q3+q4)*(1.0/5.0)-qd2*sin(q4+q5)*(2.0/2.5E1)-qd4*sin(q3+q4)*(1.0/5.0)-qd3*sin(q4+q5)*(2.0/2.5E1)-qd4*sin(q4+q5)*(2.0/2.5E1)-qd5*sin(q4+q5)*(2.0/2.5E1)-qd1*sin(q4)*(1.0/5.0)-qd2*sin(q4)*(1.0/5.0)-qd3*sin(q4)*(1.0/5.0)-qd4*sin(q4)*(1.0/5.0)-qd5*sin(q5)*(2.0/2.5E1)-qd1*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q2+q3+q4)*(1.0/5.0)-qd3*sin(q2+q3+q4)*(1.0/5.0)-qd1*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q2+q3+q4)*(1.0/5.0)-qd2*sin(q3+q4+q5)*(2.0/2.5E1)-qd3*sin(q3+q4+q5)*(2.0/2.5E1)-qd4*sin(q3+q4+q5)*(2.0/2.5E1)-qd5*sin(q3+q4+q5)*(2.0/2.5E1);
        C[4,0] = (sin(q3+q4+q5)+sin(q2+q3+q4+q5)+sin(q4+q5)+sin(q5))*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

        C[0,1] = qd1*(sin(q2+q3+q4)*(1.0/5.0)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3)*(7.0/2.5E1)+sin(q2)*(9.0/2.5E1))-qd3*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
        C[1,1] = qd3*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(-1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
        C[2,1] = -qd3*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q3)*(7.0/2.5E1))-qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)-qd4*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
        C[3,1] = qd5*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(-1.0/2.0)-qd4*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))-qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0);
        C[4,1] = (sin(q3+q4+q5)+sin(q4+q5)+sin(q5))*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

        C[0,2] = qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd4*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd1*(sin(q2+q3+q4)*(1.0/5.0)+sin(q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3)*(7.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q3)*(7.0/2.5E1));
        C[1,2] = qd2*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q3)*(7.0/2.5E1))+qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q3)*(1.4E1/2.5E1))*(1.0/2.0)-qd4*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0);
        C[2,2] = qd4*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(-1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0);
        C[3,2] = qd1*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(-1.0/2.0)-qd4*(sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))-qd2*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0);
        C[4,2] = (sin(q4+q5)+sin(q5))*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

        C[0,3] = qd1*sin(q2+q3+q4+q5)*(2.0/2.5E1)+qd1*sin(q3+q4)*(1.0/5.0)+qd2*sin(q3+q4)*(1.0/5.0)+qd1*sin(q4+q5)*(2.0/2.5E1)+qd2*sin(q4+q5)*(2.0/2.5E1)+qd3*sin(q4+q5)*(2.0/2.5E1)+qd1*sin(q4)*(1.0/5.0)+qd2*sin(q4)*(1.0/5.0)+qd3*sin(q4)*(1.0/5.0)-qd5*sin(q5)*(2.0/2.5E1)+qd1*sin(q2+q3+q4)*(1.0/5.0)+qd1*sin(q3+q4+q5)*(2.0/2.5E1)+qd2*sin(q3+q4+q5)*(2.0/2.5E1);
        C[1,3] = qd2*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q3+q4)*(1.0/5.0)+sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))+qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q3+q4)*(2.0/5.0)+sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)+qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*sin(q5)*(2.0/2.5E1);
        C[2,3] = qd3*(sin(q4+q5)*(2.0/2.5E1)+sin(q4)*(1.0/5.0))+qd1*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)+qd2*(sin(q4+q5)*(4.0/2.5E1)+sin(q4)*(2.0/5.0))*(1.0/2.0)-qd5*sin(q5)*(2.0/2.5E1);
        C[3,3] = qd5*sin(q5)*(-2.0/2.5E1);
        C[4,3] = sin(q5)*(qd1+qd2+qd3+qd4+qd5)*(-2.0/2.5E1);

        C[0,4] = qd1*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q2+q3+q4+q5)*(2.0/2.5E1)+sin(q4+q5)*(2.0/2.5E1)+sin(q5)*(2.0/2.5E1))+qd2*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd4*sin(q5)*(2.0/2.5E1);
        C[1,4] = qd2*(sin(q3+q4+q5)*(2.0/2.5E1)+sin(q4+q5)*(2.0/2.5E1)+sin(q5)*(2.0/2.5E1))+qd1*(sin(q3+q4+q5)*(4.0/2.5E1)+sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd3*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd4*sin(q5)*(2.0/2.5E1);
        C[2,4] = qd3*(sin(q4+q5)*(2.0/2.5E1)+sin(q5)*(2.0/2.5E1))+qd1*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd2*(sin(q4+q5)*(4.0/2.5E1)+sin(q5)*(4.0/2.5E1))*(1.0/2.0)+qd4*sin(q5)*(2.0/2.5E1);
        C[3,4] = sin(q5)*(qd1+qd2+qd3+qd4)*(2.0/2.5E1);
        C[4,4] = 0.0;

        return C

    #void gaussjordan(const double* inMatrix, double* outMatrix, int dim){
    def gaussjordan(self, inMatrix):
        dim1, dim2 = inMatrix.shape
        if dim1 != dim2:
            print "ERROR: gaussjordan inMatrix.shape:", inMatrix.shape
            exit(0)
        dim = dim1
        inMatrixCopy = copy.deepcopy(inMatrix)
        outMatrix = np.matrix(np.identity(dim))
 
        for diagIndex in range(dim):
            # determine diagonal factor
            diagFactor = inMatrixCopy[diagIndex,diagIndex]

            # divide column entries by diagonal factor
            for iCol in range(dim):
# TODO: check if indices order is ok
                inMatrixCopy[diagIndex,iCol] /= diagFactor
                outMatrix[diagIndex,iCol] /= diagFactor
 
            # perform line-by-line elimination
            for iRow in range(dim):
                if iRow != diagIndex:
# TODO: check if indices order is ok
                    tmpFactor = inMatrixCopy[iRow,diagIndex]

                    for iCol in range(dim):
# TODO: check if indices order is ok
                        inMatrixCopy[iRow,iCol]  -= inMatrixCopy[diagIndex,iCol]*tmpFactor
                        outMatrix[iRow,iCol] -= outMatrix[diagIndex,iCol]*tmpFactor
        return outMatrix

    def computeM(self, q):
        if type(q) == np.ndarray:
            q = np.matrix(q).transpose()
        self.M = self.inertia(q)
        self.Minv = self.gaussjordan(self.M)

    # Planar5DOF_accel(double QDD[][5], const double* input1, const double* input2, const double* input3)
    def accel(self, q, dq, trq):
        if type(q) == np.ndarray:
            q = np.matrix(q).transpose()
        if type(dq) == np.ndarray:
            dq = np.matrix(dq).transpose()
        if type(trq) == np.ndarray:
            trq = np.matrix(trq).transpose()

        # call the computational routines
        I = self.M
        Iinv = self.Minv
        C = self.coriolis(q, dq);
         
        # fill temporary vector
        tmpTau = C * dq
        for iCol in range(5):
            tmpTau[iCol,0] = trq[iCol,0] -  tmpTau[iCol,0] - dq[iCol] * 0.2

        # compute acceleration
        QDD = Iinv * tmpTau

        return QDD


