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

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <boost/bind.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Path.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>

#include <stdlib.h>

#include "Eigen/Dense"

#include <collision_convex_model/collision_convex_model.h>
#include "kin_model/kin_model.h"
#include "planer_utils/marker_publisher.h"
#include "planer_utils/utilities.h"
#include "planer_utils/random_uniform.h"

class TestOmpl {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    MarkerPublisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

public:
    TestOmpl() :
        nh_(),
        PI(3.141592653589793),
        markers_pub_(nh_)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
    }

    ~TestOmpl() {
    }

    void stateOmplToEigen(const ompl::base::State *s, Eigen::VectorXd &x, int ndof) {
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            x(q_idx) = s->as<ompl::base::RealVectorStateSpace::StateType >()->operator[](q_idx);
        }
    }

    bool isStateValid(const ompl::base::State *s, const boost::shared_ptr<self_collision::CollisionModel > &col_model, const boost::shared_ptr<KinematicModel > &kin_model, int ndof) {
        Eigen::VectorXd x(ndof);
        stateOmplToEigen(s, x, ndof);

        std::vector<self_collision::CollisionInfo> link_collisions;
        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
        // calculate forward kinematics for all links
        for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
            kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
        }
        self_collision::getCollisionPairs(col_model, links_fk, 0.05, link_collisions);
        if (link_collisions.size() > 0) {
            return false;
        }

        return true;
    }

    void spin() {

        // initialize random seed
        srand(time(NULL));

        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

        //
        // collision model
        //
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);
	    col_model->parseSRDF(robot_semantic_description_str);
        col_model->generateCollisionPairs();

        // external collision objects - part of virtual link connected to the base link
        self_collision::Link::VecPtrCollision col_array;
//        col_array.push_back( self_collision::createCollisionCapsule(0.2, 0.3, KDL::Frame(KDL::Vector(1, 0.5, 0))) );
        col_array.push_back( self_collision::createCollisionCapsule(0.05, 0.3, KDL::Frame(KDL::Rotation::RotX(90.0/180.0*PI), KDL::Vector(1, 0.2, 0))) );
        col_array.push_back( self_collision::createCollisionCapsule(0.05, 0.2, KDL::Frame(KDL::Rotation::RotZ(90.0/180.0*PI)*KDL::Rotation::RotX(90.0/180.0*PI), KDL::Vector(0.9, 0.35, 0))) );
        if (!col_model->addLink("env_link", "base", col_array)) {
            ROS_ERROR("ERROR: could not add external collision objects to the collision model");
            return;
        }
        col_model->generateCollisionPairs();

        //
        // robot state
        //
        std::vector<std::string > joint_names;
        joint_names.push_back("0_joint");
        joint_names.push_back("1_joint");
        joint_names.push_back("2_joint");
        joint_names.push_back("3_joint");
        joint_names.push_back("4_joint");

        int ndof = joint_names.size();

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = -0.1;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
        }

        Eigen::VectorXd saved_q, saved_dq, saved_ddq;
        saved_q.resize( ndof );
        saved_dq.resize( ndof );
        saved_ddq.resize( ndof );

        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            saved_q[q_idx] = q[q_idx];
            saved_dq[q_idx] = dq[q_idx];
            saved_ddq[q_idx] = ddq[q_idx];
        }

        std::string effector_name = "effector";
        int effector_idx = col_model->getLinkIndex(effector_name);

        //
        // kinematic model
        //
        boost::shared_ptr<KinematicModel > kin_model( new KinematicModel(robot_description_str, joint_names) );

        std::vector<KDL::Frame > links_fk(col_model->getLinksCount());

        // joint limits
        Eigen::VectorXd lower_limit(ndof), upper_limit(ndof), limit_range(ndof), max_trq(ndof);
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator name_it = joint_names.begin(); name_it != joint_names.end(); name_it++, q_idx++) {

            if (!col_model->getJointLimits( (*name_it), lower_limit[q_idx], upper_limit[q_idx] )) {
                ROS_ERROR("ERROR: could not find joint with name %s", name_it->c_str() );
                return;
            }
            limit_range[q_idx] = 10.0/180.0*PI;
            max_trq[q_idx] = 10.0;
        }

        //
        // ompl
        //

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace());
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            space->as<ompl::base::RealVectorStateSpace>()->addDimension(joint_names[q_idx], lower_limit(q_idx), upper_limit(q_idx));
        }

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
        si->setStateValidityChecker( boost::bind(&TestOmpl::isStateValid, this, _1, col_model, kin_model, ndof) );
        si->setStateValidityCheckingResolution(0.03);
        si->setup();

        while (ros::ok()) {
            ompl::base::ScopedState<> start(space);

            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                start[q_idx] = q(q_idx);
            }

            ompl::base::ScopedState<> goal(space);

            while (true) {
                goal.random();
                if (isStateValid(goal.get(), col_model, kin_model, ndof)) {
                    break;
                }
            }

            Eigen::VectorXd xe(ndof);
            stateOmplToEigen(goal.get(), xe, ndof);
            KDL::Frame T_B_E;
            kin_model->calculateFk(T_B_E, col_model->getLinkName(effector_idx), xe);
            publishTransform(br, T_B_E, "effector_dest");

            ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
            pdef->clearStartStates();
            pdef->setStartAndGoalStates(start, goal);

            ompl::base::PlannerPtr planner(new ompl::geometric::LBTRRT(si));
//            ompl::base::PlannerPtr planner(new ompl::geometric::RRTstar(si));
//            ompl::base::PlannerPtr planner(new ompl::geometric::RRTConnect(si));
            planner->setProblemDefinition(pdef);
            planner->setup();

            ompl::base::PlannerStatus status = planner->solve(2.0);

            if (status) {
                std::cout << "found solution" << std::endl;
                ompl::base::PathPtr path = pdef->getSolutionPath();
                std::cout << "path length: " << path->length() << std::endl;
                boost::shared_ptr<ompl::geometric::PathGeometric > ppath = boost::static_pointer_cast<ompl::geometric::PathGeometric >(path);

                std::list<Eigen::VectorXd > path2;
                for (int i = 0; i< ppath->getStateCount(); i++) {
                    ompl::base::State *s = ppath->getState(i);
                    Eigen::VectorXd x(ndof);
                    stateOmplToEigen(s, x, ndof);
                    path2.push_back(x);
                }

                
                for (double f = 0.0; f < 1.0; f += 0.01/path->length()) {
                    Eigen::VectorXd x(ndof);
                    getPointOnPath(path2, f, x);

                    std::vector<KDL::Frame > links_fk(col_model->getLinksCount());
                    // calculate forward kinematics for all links
                    for (int l_idx = 0; l_idx < col_model->getLinksCount(); l_idx++) {
                        kin_model->calculateFk(links_fk[l_idx], col_model->getLinkName(l_idx), x);
                    }

                    publishJointState(joint_state_pub_, x, joint_names);
                    int m_id = 0;
                    m_id = addRobotModelVis(markers_pub_, m_id, col_model, links_fk);
                    markers_pub_.publish();

                    ros::spinOnce();
                    ros::Duration(0.01).sleep();
                }

                q = xe;
                getchar();

            }
            else {
                std::cout << "solution not found" << std::endl;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_ompl");
    TestOmpl test;
    test.spin();
    return 0;
}

