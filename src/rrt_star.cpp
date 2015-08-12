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

#include "rrt_star.h"

#include "random_uniform.h"

    RRTStar::RRTStar(int ndof,
            boost::function<bool(const Eigen::VectorXd &x)> collision_func,
            boost::function<double(const Eigen::VectorXd &x, const Eigen::VectorXd &y)> costLine_func,
            boost::function<void(Eigen::VectorXd &sample)> sampleSpace_func,
            double collision_check_step, double steer_dist, double near_dist) :
        ndof_(ndof),
        collision_func_(collision_func),
        costLine_func_(costLine_func),
        sampleSpace_func_(sampleSpace_func),
        collision_check_step_(collision_check_step),
        steer_dist_(steer_dist),
        near_dist_(near_dist)
    {
    }

    bool RRTStar::isStateValid(const Eigen::VectorXd &x) const {
        return !collision_func_(x);
    }

    void RRTStar::sampleSpace(Eigen::VectorXd &sample) const {
        sampleSpace_func_(sample);
    }

    bool RRTStar::sampleFree(Eigen::VectorXd &sample_free) const {
        Eigen::VectorXd x(ndof_);
        for (int i=0; i < 100; i++) {
            sampleSpace(x);
            if (isStateValid(x)) {
                sample_free = x;
                return true;
            }
        }
        return false;
    }

    int RRTStar::nearest(const Eigen::VectorXd &x) const {
        double min_dist = -1.0;
        int min_idx = -1;
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - x).norm();
            if (min_idx < 0 || dist < min_dist) {
                min_dist = dist;
                min_idx = v_it->first;
            }
        }
        return min_idx;
    }

    void RRTStar::steer(const Eigen::VectorXd &x_from, const Eigen::VectorXd &x_to, double steer_dist, Eigen::VectorXd &x) const {
        Eigen::VectorXd v = x_to - x_from;
        if (v.norm() <= steer_dist) {
            x = x_to;
        }
        else {
            x = x_from + steer_dist * v / v.norm();
        }
    }

    bool RRTStar::collisionFree(const Eigen::VectorXd &x_from, const Eigen::VectorXd &x_to) const {
        Eigen::VectorXd v = x_to - x_from;
        double dist = v.norm();
        v = v / dist;
        double progress = 0.0;
        while (true) {
            progress += collision_check_step_;
            bool end = false;
            if (progress > dist) {
                progress = dist;
                end = true;
            }
            Eigen::VectorXd x = x_from + v * progress;
            if (!isStateValid(x)) {
                return false;
            }
            if (end) {
                break;
            }
        }
        return true;
    }

    void RRTStar::near(const Eigen::VectorXd &x, double near_dist, std::list<int > &q_near_idx_list) const {
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - x).norm();
            if (dist <= near_dist) {
                q_near_idx_list.push_back(v_it->first);
            }
        }
    }

    double RRTStar::costLine(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const {
        return costLine_func_(x1, x2);
    }

    double RRTStar::costLine(int x1_idx, int x2_idx) const {
        return costLine(V_.find(x1_idx)->second, V_.find(x2_idx)->second);
    }


    double RRTStar::cost(int q_idx) const {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            return 0.0;
        }
        int q_parent_idx = e_it->second;
        return costLine(q_idx, q_parent_idx) + cost(q_parent_idx);
    }

    void RRTStar::getPath(int q_idx, std::list<int > &path) const {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            path.clear();
            path.push_back(q_idx);
        }
        else {
            int q_parent_idx = e_it->second;
            getPath(q_parent_idx, path);
            path.push_back(q_idx);
        }
    }

    void RRTStar::plan(const Eigen::VectorXd &start, const Eigen::VectorXd &goal, double goal_tolerance, std::list<Eigen::VectorXd > &path) {
        V_.clear();
        E_.clear();
        path.clear();

        int q_new_idx = 0;
        V_[0] = start;

        for (int step = 0; step < 50000; step++) {
            bool sample_goal = randomUniform(0,1) < 0.05;
            Eigen::VectorXd q_rand(ndof_);

            if (sample_goal) {
                q_rand = goal;
            }
            else {
                if (!sampleFree(q_rand)) {
                    std::cout << "ERROR: RRTStar::plan: could not sample free space" << std::endl;
                    return;
                }
            }

            int q_nearest_idx = nearest(q_rand);
            const Eigen::VectorXd &q_nearest = V_.find(q_nearest_idx)->second;
            Eigen::VectorXd q_new(ndof_);
            steer(q_nearest, q_rand, steer_dist_, q_new);

            if (collisionFree(q_nearest, q_new)) {

                bool isGoal = (q_new - goal).norm() < goal_tolerance;

                std::list<int > q_near_idx_list;
                near(q_new, near_dist_, q_near_idx_list);
                double min_cost = cost(q_nearest_idx);
                int min_idx = q_nearest_idx;                
                for (std::list<int >::const_iterator qi_it = q_near_idx_list.begin(); qi_it != q_near_idx_list.end(); qi_it++) {
                    int q_idx = *qi_it;
                    double c = cost(q_idx);
                    if (min_idx == -1 || min_cost > c && collisionFree(V_.find(q_idx)->second, q_new)) {
                        min_idx = q_idx;
                        min_cost = c;
                    }
                }

                q_new_idx++;
                V_[q_new_idx] = q_new;
                E_[q_new_idx] = min_idx;


                double cost_q_new = cost(q_new_idx);
                for (std::list<int >::const_iterator qi_it = q_near_idx_list.begin(); qi_it != q_near_idx_list.end(); qi_it++) {
                    int q_near_idx = *qi_it;
                    Eigen::VectorXd q_near = V_.find(q_near_idx)->second;
                    if (cost_q_new + costLine(q_new, q_near) < cost(q_near_idx)) {
                        bool col_free = collisionFree(q_new, q_near);
                        if (col_free) {
                                int q_parent_idx = E_[q_near_idx];
                                E_[q_near_idx] = q_new_idx;
                        }
                    }
                }
            }
        }

        double min_cost = 0.0;
        int min_goal_idx = -1;
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - goal).norm();
            double c = cost(v_it->first);
            if (dist < goal_tolerance && (min_goal_idx < 0 || min_cost > c)) {
                min_cost = c;
                min_goal_idx = v_it->first;
            }
        }

        if (min_goal_idx == -1) {
            // path not found
            return;
        }

        std::list<int > idx_path;
        getPath(min_goal_idx, idx_path);
        for (std::list<int >::const_iterator p_it = idx_path.begin(); p_it != idx_path.end(); p_it++) {
            path.push_back(V_.find(*p_it)->second);
        }
    }

    int RRTStar::addTreeMarker(MarkerPublisher &markers_pub, int m_id) const {
        std::vector<std::pair<KDL::Vector, KDL::Vector > > vec_arr;
        for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
            const Eigen::VectorXd &x1 = V_.find(e_it->first)->second;
            const Eigen::VectorXd &x2 = V_.find(e_it->second)->second;
            KDL::Vector pos1(x1(0), x1(1), 0), pos2(x2(0), x2(1), 0);
            vec_arr.push_back( std::make_pair(pos1, pos2) );
            m_id = markers_pub.addVectorMarker(m_id, pos1, pos2, 0, 0.7, 0, 0.5, 0.01, "base");
        }

        const Eigen::VectorXd &xs = V_.find(0)->second;
        KDL::Vector pos(xs(0), xs(1), 0);
        m_id = markers_pub.addSinglePointMarker(m_id, pos, 0, 1, 0, 1, 0.05, "base");
        return m_id;
    }

