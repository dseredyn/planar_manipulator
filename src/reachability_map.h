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

#ifndef REACHABILITY_MAP_H__
#define REACHABILITY_MAP_H__

#include "Eigen/Dense"

#include "reachability_map.h"
#include <collision_convex_model/collision_convex_model.h>
#include "kin_model.h"

class ReachabilityMap {
public:
    ReachabilityMap(double voxel_size, int dim);

    ~ReachabilityMap();

    void generate(const KinematicModel &kin_model, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::string &effector_name, int ndof, const Eigen::VectorXd &lower_limit, const Eigen::VectorXd &upper_limit);

    double getValue(const Eigen::VectorXd &x) const;

    void addPenalty(const Eigen::VectorXd &x);
    void resetPenalty();

protected:

    int getIndex(const Eigen::VectorXd &x) const;

    double voxel_size_;
    int dim_;
    int max_value_;
    Eigen::VectorXd ep_min_, ep_max_;
    std::vector<int > r_map_;
    std::vector<int > p_map_;
    std::vector<int > steps_;
};

#endif  // REACHABILITY_MAP_H__

