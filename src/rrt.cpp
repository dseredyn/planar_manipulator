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

#include "rrt.h"

#include "planer_utils/random_uniform.h"

    RRT::RRT(int ndof,
            boost::function<bool(const KDL::Frame &x)> collision_func,
            boost::function<void(KDL::Frame &sample)> sampleSpace_func,
            double collision_check_step, double steer_dist, double near_dist,
            const boost::shared_ptr<KinematicModel> &kin_model,
            const std::string &effector_name,
            boost::shared_ptr<DynamicsSimulatorHandPose> &sim) :
        ndof_(ndof),
        collision_func_(collision_func),
        sampleSpace_func_(sampleSpace_func),
        collision_check_step_(collision_check_step),
        steer_dist_(steer_dist),
        near_dist_(near_dist),
        kin_model_(kin_model),
        effector_name_(effector_name),
        sim_(sim)
    {
    }

    bool RRT::isPoseValid(const KDL::Frame &x) const {
        return !collision_func_(x);
    }

    void RRT::sampleSpace(KDL::Frame &sample) const {
        sampleSpace_func_(sample);
    }

    bool RRT::sampleFree(KDL::Frame &sample_free) const {
        KDL::Frame x;
        for (int i=0; i < 100; i++) {
            sampleSpace(x);
            if (isPoseValid(x)) {
                sample_free = x;
                return true;
            }
        }
        return false;
    }

    int RRT::nearest(const KDL::Frame &x) const {
        double min_dist = -1.0;
        int min_idx = -1;
        for (std::map<int, RRTState >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            KDL::Twist diff( KDL::diff(v_it->second.T_B_E_, x, 1.0) );
            double dist = diff.vel.Norm() + diff.rot.Norm();
            if (min_idx < 0 || dist < min_dist) {
                min_dist = dist;
                min_idx = v_it->first;
            }
        }
        return min_idx;
    }

    void RRT::steer(const KDL::Frame &x_from, const KDL::Frame &x_to, double steer_dist_lin, double steer_dist_rot, KDL::Frame &x) const {
        KDL::Twist diff( KDL::diff(x_from, x_to, 1.0) );

        double dist_lin = diff.vel.Norm();

        if (dist_lin > steer_dist_lin) {
            diff.vel = steer_dist_lin * diff.vel / dist_lin;
            diff.rot = steer_dist_lin * diff.rot / dist_lin;
        }

        double dist_rot = diff.rot.Norm();
        if (dist_rot > steer_dist_rot) {
            diff.vel = steer_dist_rot * diff.vel / dist_rot;
            diff.rot = steer_dist_rot * diff.rot / dist_rot;
        }

        x = KDL::addDelta(x_from, diff, 1.0);
    }
/*
    bool RRT::collisionFree(const Eigen::VectorXd &q_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, KDL::Frame &x_to_out) const {

        Eigen::VectorXd dq(ndof_), ddq(ndof_);
        for (int q_idx = 0; q_idx < ndof_; q_idx++) {
            dq(q_idx) = 0.0;
            ddq(q_idx) = 0.0;
        }
        sim_->setTarget(x_to);
        sim_->setState(q_from, dq, ddq);
        KDL::Twist diff_target = KDL::diff(x_from, x_to, 1.0);
        if (try_idx == 1) {
            double angle = diff_target.rot.Norm();
            diff_target.rot = (angle - 360.0/180.0*3.141592653589793) * diff_target.rot / angle;
        }
        else if (try_idx == 2) {
            double angle = diff_target.rot.Norm();
            diff_target.rot = (angle - 2.0*360.0/180.0*3.141592653589793) * diff_target.rot / angle;
        }

        for (int loop_counter = 0; loop_counter < 1500; loop_counter++) {
            double f_path = static_cast<double >(loop_counter)/800.0;
            if (f_path > 1.0) {
                f_path = 1.0;
            }
            KDL::Vector pt;
            pt = x_to.p * f_path + x_from.p * (1.0-f_path);

            KDL::Rotation target_rot = KDL::addDelta(x_from, diff_target, f_path).M;

            Eigen::VectorXd q(ndof_), dq(ndof_), ddq(ndof_);
            sim_->getState(q, dq, ddq);

            KDL::Frame r_HAND_current;
            kin_model_->calculateFk(r_HAND_current, effector_name_, q);

            KDL::Twist goal_diff( KDL::diff(r_HAND_current, x_to, 1.0) );
            if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                q_to = q;
                x_to_out = r_HAND_current;
                return true;
            }

            KDL::Frame target_pos(target_rot, pt);
            KDL::Twist diff = KDL::diff(r_HAND_current, target_pos, 1.0);

            sim_->oneStep(diff);
        }
        return false;
    }
*/

    bool RRT::collisionFree(const Eigen::VectorXd &q_from, const KDL::Frame &x_from, const KDL::Frame &x_to, int try_idx, Eigen::VectorXd &q_to, KDL::Frame &x_to_out) const {

        Eigen::VectorXd dq(ndof_), ddq(ndof_);
        for (int q_idx = 0; q_idx < ndof_; q_idx++) {
            dq(q_idx) = 0.0;
            ddq(q_idx) = 0.0;
        }
        sim_->setTarget(x_to);
        sim_->setState(q_from, dq, ddq);
        KDL::Twist diff_target = KDL::diff(x_from, x_to, 1.0);

        for (int loop_counter = 0; loop_counter < 3000; loop_counter++) {
            Eigen::VectorXd q(ndof_), dq(ndof_), ddq(ndof_);
            sim_->getState(q, dq, ddq);

            KDL::Frame r_HAND_current;
            kin_model_->calculateFk(r_HAND_current, effector_name_, q);

            KDL::Twist goal_diff( KDL::diff(r_HAND_current, x_to, 1.0) );
            if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                q_to = q;
                x_to_out = r_HAND_current;
                return true;
            }

            sim_->oneStep();
        }
        return false;
    }

    double RRT::costLine(const KDL::Frame &x1, const KDL::Frame &x2) const {
        KDL::Twist diff( KDL::diff(x1, x2, 1.0) );
        return diff.vel.Norm() + diff.rot.Norm();
    }

    double RRT::costLine(int x1_idx, int x2_idx) const {
        return costLine(V_.find(x1_idx)->second.T_B_E_, V_.find(x2_idx)->second.T_B_E_);
    }


    double RRT::cost(int q_idx) const {
        std::map<int, int >::const_iterator e_it = E_.find(q_idx);
        if (e_it == E_.end()) {
            return 0.0;
        }
        int q_parent_idx = e_it->second;
        return costLine(q_idx, q_parent_idx) + cost(q_parent_idx);
    }

    void RRT::getPath(int q_idx, std::list<int > &path) const {
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

    void RRT::plan(const Eigen::VectorXd &start, const KDL::Frame &x_goal, double goal_tolerance, std::list<KDL::Frame > &path_x, std::list<Eigen::VectorXd > &path_q, MarkerPublisher &markers_pub) {
        V_.clear();
        E_.clear();
        path_x.clear();
        path_q.clear();

        int q_new_idx = 0;

        RRTState state_start;
        kin_model_->calculateFk(state_start.T_B_E_, effector_name_, start);
        state_start.q_ = start;
        V_[0] = state_start;

        bool goal_found = false;

        int m_id = 0;
        for (int step = 0; step < 1200; step++) {
            bool sample_goal = randomUniform(0,1) < 0.05;
            KDL::Frame x_rand;

            if (sample_goal) {
                x_rand = x_goal;
            }
            else {
                if (!sampleFree(x_rand)) {
                    std::cout << "ERROR: RRT::plan: could not sample free space" << std::endl;
                    return;
                }
            }

            // get the closest pose
            int x_nearest_idx = nearest(x_rand);
            RRTState state_nearest( V_.find(x_nearest_idx)->second );
            const KDL::Frame &x_nearest = state_nearest.T_B_E_;
            KDL::Frame x_new;
            // get the new pose
            steer(x_nearest, x_rand, 2.0, 170.0/180.0*3.1415, x_new);

//            std::cout << x_nearest.p[0] << " " << x_nearest.p[1] << " " << x_nearest.p[2] << std::endl;
//            std::cout << x_new.p[0] << " " << x_new.p[1] << " " << x_new.p[2] << std::endl;

            Eigen::VectorXd q_new(ndof_);
            bool added_new = false;
            KDL::Frame T_B_E;
            if (collisionFree(state_nearest.q_, x_nearest, x_new, 0, q_new, T_B_E)) {

                        added_new = true;
                        RRTState state_new;
                        state_new.T_B_E_ = T_B_E;//x_new;
                        state_new.q_ = q_new;
                        q_new_idx++;
                        V_[q_new_idx] = state_new;
                        E_[q_new_idx] = x_nearest_idx;
                        m_id = markers_pub.addVectorMarker(q_new_idx, x_nearest.p, T_B_E.p, 0, 0.7, 0, 0.5, 0.01, "base");

                        KDL::Twist goal_diff( KDL::diff(T_B_E, x_goal, 1.0) );
                        if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                            goal_found = true;
                            std::cout << "goal found" << std::endl;
                        }
            }
            else {

                KDL::Twist goal_diff( KDL::diff(x_new, x_goal, 1.0) );
                if (goal_diff.vel.Norm() < 0.03 && goal_diff.rot.Norm() < 10.0/180.0*3.1415) {
                    std::list<int> nodes_to_remove_list;
                    getPath(x_nearest_idx, nodes_to_remove_list);
                    if (nodes_to_remove_list.size() > 1) {
                        nodes_to_remove_list.pop_front();
                        std::set<int> nodes_to_remove;

                        int removed_nodes_count = 0;
                        for (std::list<int>::const_iterator it = nodes_to_remove_list.begin(); it != nodes_to_remove_list.end(); it++) {
                            nodes_to_remove.insert( (*it) );
                        }

                        while (nodes_to_remove.size() > 0) {
                            std::set<int> orphan_nodes;
                            for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
                                if (nodes_to_remove.find(e_it->second) != nodes_to_remove.end()) {
                                    orphan_nodes.insert(e_it->first);
                                }
                            }

                            for (std::set<int>::const_iterator it = nodes_to_remove.begin(); it != nodes_to_remove.end(); it++) {
                                V_.erase( (*it) );
                                E_.erase( (*it) );
                                markers_pub.addEraseMarkers( (*it), (*it)+1);
                                removed_nodes_count++;
                            }
                            nodes_to_remove = orphan_nodes;
                        }
                        std::cout << "removed nodes: " << removed_nodes_count << std::endl;
                    }
                }

                if (sample_goal) {
                    // remove the nearest node to the goal
//                    V_.erase(x_nearest_idx);
//                    E_.erase(x_nearest_idx);
//                    markers_pub.addEraseMarkers(x_nearest_idx, x_nearest_idx+1);
                }
            }

            if (added_new) {
                std::cout << "step " << step << "  nearest_idx " << x_nearest_idx << std::endl;
            }

            markers_pub.publish();
            ros::spinOnce();
//            getchar();
            if (goal_found) {
                break;
            }
        }
/*
        double min_cost = 0.0;
        int min_goal_idx = -1;
        for (std::map<int, Eigen::VectorXd >::const_iterator v_it = V_.begin(); v_it != V_.end(); v_it++) {
            double dist = (v_it->second - x_goal).norm();
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
*/

        if (goal_found) {
            
            std::list<int > idx_path;
            getPath(q_new_idx, idx_path);
            for (std::list<int >::const_iterator p_it = idx_path.begin(); p_it != idx_path.end(); p_it++) {
                const RRTState &current = V_.find(*p_it)->second;
                path_q.push_back(current.q_);
                path_x.push_back(current.T_B_E_);
            }
        }

/*
        int q_vec_idx = V_[q_new_idx].q_vec_.size()-1;
        std::list<int > idx_path;
        getPath(q_new_idx, idx_path);
        for (std::list<int >::const_reverse_iterator p_it = idx_path.rbegin(); p_it != idx_path.rend(); p_it++) {
//            std::list<int >::const_reverse_iterator p_it2 = p_it;
//            p_it2++;
            const RRTState &current = V_.find(*p_it)->second;

//            if (p_it2 == idx_path.rend()) {
//                path_q.push_back(current.q_vec_[q_vec_idx]->second);
//            }
//            else {
            path_q.push_back(current.q_vec_[q_vec_idx].second);
            q_vec_idx = current.q_vec_[q_vec_idx].first;
//            }
        }

        std::reverse(path_q.begin(), path_q.end());
*/
/*
        for (std::list<int >::const_iterator p_it = idx_path.begin(); p_it != idx_path.end(); p_it++) {
            const RRTState &current = V_.find(*p_it)->second;
            if (p_it == idx_path.begin()) {
                path_q.push_back(current.q_vec_[0]);
            }
            else {
                std::list<int >::const_iterator p_it_prev = p_it;
                p_it_prev--;
                const RRTState &prev = V_.find(*p_it_prev)->second;
                path_q.push_back(current.q_vec_[0]);
            }
        }
*/
    }

    int RRT::addTreeMarker(MarkerPublisher &markers_pub, int m_id) const {
/*        std::vector<std::pair<KDL::Vector, KDL::Vector > > vec_arr;
        for (std::map<int, int >::const_iterator e_it = E_.begin(); e_it != E_.end(); e_it++) {
            const Eigen::VectorXd &x1 = V_.find(e_it->first)->second.T_B_E;
            const Eigen::VectorXd &x2 = V_.find(e_it->second)->second.T_B_E;
            KDL::Vector pos1(x1(0), x1(1), 0), pos2(x2(0), x2(1), 0);
            vec_arr.push_back( std::make_pair(pos1, pos2) );
            m_id = markers_pub.addVectorMarker(m_id, pos1, pos2, 0, 0.7, 0, 0.5, 0.01, "base");
        }

        const Eigen::VectorXd &xs = V_.find(0)->second;
        KDL::Vector pos(xs(0), xs(1), 0);
        m_id = markers_pub.addSinglePointMarker(m_id, pos, 0, 1, 0, 1, 0.05, "base");
*/
        return m_id;
    }

