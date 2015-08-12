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

#ifndef PLANAR_COLLISION_H__
#define PLANAR_COLLISION_H__

#include "Eigen/Dense"

#include <collision_convex_model/collision_convex_model.h>
#include "task_col.h"

    void distancePoints(const KDL::Vector &pt1, const KDL::Vector &pt2, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2);

    void distanceLinePoint(const KDL::Vector &lineA, const KDL::Vector &lineB, const KDL::Vector &pt, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2);

    void distancePointLine(const KDL::Vector &pt, const KDL::Vector &lineA, const KDL::Vector &lineB, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2);

    void distanceLines(const KDL::Vector &l1A, const KDL::Vector &l1B, const KDL::Vector &l2A, const KDL::Vector &l2B, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2);

    bool checkCollision(const boost::shared_ptr<self_collision::Collision> &col1, const boost::shared_ptr<self_collision::Collision> &col2,
                        const KDL::Frame &T_B_L1, const KDL::Frame &T_B_L2, double activation_dist,
                        double &dist, KDL::Vector &p1_B, KDL::Vector &p2_B, KDL::Vector &n1_B, KDL::Vector &n2_B);

    void getCollisionPairs(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk, double activation_dist, std::vector<CollisionInfo> &link_collisions);

    bool checkSelfCollision(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk, double activation_dist);

    void getRepulsiveForces(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                               double activation_dist, const KDL::Vector &center, std::vector<CollisionInfo> &link_collisions);

#endif  // PLANAR_COLLISION_H__

