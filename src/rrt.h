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

#ifndef RRT_H__
#define RRT_H__

#include "Eigen/Dense"

#include "planer_utils/marker_publisher.h"
#include "kin_model/kin_model.h"
#include "planer_utils/simulator.h"

class RRTState {
public:
    KDL::Frame T_B_E_;
    Eigen::VectorXd q_;
//    std::vector<std::pair<int, Eigen::VectorXd > > q_vec_;
};

class RRT {
public:

    RRT(int ndof,
            boost::function<bool(const KDL::Frame &x)> collision_func,
            boost::function<void(KDL::Frame &sample)> sampleSpace_func,
            double collision_check_step, double steer_dist, double near_dist,
            const boost::shared_ptr<KinematicModel> &kin_model,
            const std::string &effector_name,
            boost::shared_ptr<DynamicsSimulatorHandPose> &sim);

    bool isPoseValid(const KDL::Frame &x) const;

    void sampleSpace(KDL::Frame &sample) const;

    bool sampleFree(KDL::Frame &sample_free) const;

    int nearest(const KDL::Frame &x) const;

    void steer(const KDL::Frame &x_from, const KDL::Frame &x_to, double steer_dist_lin, double steer_dist_rot, KDL::Frame &x) const;

    bool collisionFree(const Eigen::VectorXd &q_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, KDL::Frame &x_to_out) const;

    double costLine(const KDL::Frame &x1, const KDL::Frame &x2) const;

    double costLine(int x1_idx, int x2_idx) const;

    double cost(int q_idx) const;

    void getPath(int q_idx, std::list<int > &path) const;

    void plan(const Eigen::VectorXd &start, const KDL::Frame &x_goal, double goal_tolerance, std::list<KDL::Frame > &path_x, std::list<Eigen::VectorXd > &path_q, MarkerPublisher &markers_pub);

    int addTreeMarker(MarkerPublisher &markers_pub, int m_id) const;

protected:
    boost::function<bool(const KDL::Frame &x)> collision_func_;
    boost::function<void(KDL::Frame &sample)> sampleSpace_func_;
    std::map<int, RRTState > V_;
    std::map<int, int > E_;
    double collision_check_step_;
    int ndof_;
    double steer_dist_;
    double near_dist_;
    const boost::shared_ptr<KinematicModel> &kin_model_;
    std::string effector_name_;
    boost::shared_ptr<DynamicsSimulatorHandPose> &sim_;
};

#endif  // RRT_H__

