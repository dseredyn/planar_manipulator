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

#ifndef RRT_STAR_H__
#define RRT_STAR_H__

#include "Eigen/Dense"

#include "marker_publisher.h"

class RRTStar {
public:
    RRTStar(int ndof,
            boost::function<bool(const Eigen::VectorXd &x)> collision_func,
            boost::function<double(const Eigen::VectorXd &x, const Eigen::VectorXd &y)> costLine_func,
            boost::function<void(Eigen::VectorXd &sample)> sampleSpace_func,
            double collision_check_step, double steer_dist, double near_dist);

    bool isStateValid(const Eigen::VectorXd &x) const;

    void sampleSpace(Eigen::VectorXd &sample) const;

    bool sampleFree(Eigen::VectorXd &sample_free) const;

    int nearest(const Eigen::VectorXd &x) const;

    void steer(const Eigen::VectorXd &x_from, const Eigen::VectorXd &x_to, double steer_dist, Eigen::VectorXd &x) const;

    bool collisionFree(const Eigen::VectorXd &x_from, const Eigen::VectorXd &x_to) const;

    void near(const Eigen::VectorXd &x, double near_dist, std::list<int > &q_near_idx_list) const;

    double costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const;

    double costLine(int x1_idx, int x2_idx) const;

    double cost(int q_idx) const;

    void getPath(int q_idx, std::list<int > &path) const;

    void plan(const Eigen::VectorXd &start, const Eigen::VectorXd &goal, double goal_tolerance, std::list<Eigen::VectorXd > &path);

    int addTreeMarker(MarkerPublisher &markers_pub, int m_id) const;

protected:
    boost::function<bool(const Eigen::VectorXd &x)> collision_func_;
    boost::function<double(const Eigen::VectorXd &x, const Eigen::VectorXd &y)> costLine_func_;
    boost::function<void(Eigen::VectorXd &sample)> sampleSpace_func_;
    std::map<int, Eigen::VectorXd > V_;
    std::map<int, int > E_;
    double collision_check_step_;
    int ndof_;
    double steer_dist_;
    double near_dist_;
};

#endif  // RRT_STAR_H__

