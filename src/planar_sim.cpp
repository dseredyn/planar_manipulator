#include <string>

#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include "planar5_dyn_model.h"
#include <self_collision/urdf_collision_parser.h>
#include "fk_ik.h"
#include "marker_publisher.h"

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Publisher markers_pub_;

public:
    TestDynamicModel() {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/velma_markers", 10);
    }

    ~TestDynamicModel() {
    }

    void publishJointState(const Eigen::VectorXd &q, const std::vector<std::string > &joint_names) {
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        int q_idx = 0;
        for (std::vector<std::string >::const_iterator it = joint_names.begin(); it != joint_names.end(); it++, q_idx++) {
            js.name.push_back(*it);
            js.position.push_back(q[q_idx]);
        }
        joint_state_pub_.publish(js);
    }

//    def publishTransform(self, T_B_F, frame_id):
//        wrist_pose = pm.toMsg(T_B_F)
//        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), frame_id, "base")

    int publishRobotModelVis(int m_id, const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &T) {
        for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
            boost::shared_ptr<self_collision::Link> link = col_model->links_[l_idx];
            KDL::Frame T_B_L = T[l_idx];
            for (self_collision::Link::VecPtrCollision::const_iterator it = link->collision_array.begin(); it != link->collision_array.end(); it++) {
                KDL::Frame T_B_O = T_B_L * (*it)->origin;
                if ((*it)->geometry->type == self_collision::Geometry::CONVEX) {
                    // TODO
                }
                else if ((*it)->geometry->type == self_collision::Geometry::SPHERE) {
                    self_collision::Sphere *sphere = static_cast<self_collision::Sphere* >((*it)->geometry.get());
                    m_id = publishSinglePointMarker(markers_pub_, m_id, T_B_O.p, 0, 1, 0, sphere->radius*2, "base");
                }
                else if ((*it)->geometry->type == self_collision::Geometry::CAPSULE) {
                    self_collision::Capsule *capsule = static_cast<self_collision::Capsule* >((*it)->geometry.get());
                    m_id = publishCapsule(markers_pub_, m_id, T_B_O, capsule->length, capsule->radius, "base");
                }
            }
        }
        return m_id;
    }

    void spin() {
        DynModelPlanar5 dyn_model;

        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);
	    col_model-> parseSRDF(robot_semantic_description_str);

        std::vector<std::string > joint_names;
        joint_names.push_back("0_joint");
        joint_names.push_back("1_joint");
        joint_names.push_back("2_joint");
        joint_names.push_back("3_joint");
        joint_names.push_back("4_joint");

        std::string effector_name = "effector";

        int ndof = joint_names.size();

        FkIkSolver solver(robot_description_str);//joint_names, [], None);

        FkIkSolver::Jacobian J;
        J.resize(6, ndof);

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.0;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }

        solver.getJacobian(J, effector_name, q);

        dyn_model.accel(ddq, q, dq, torque);

        std::vector<KDL::Frame > links_fk(col_model->link_count_);

        ros::Rate loop_rate(10);
        while (ros::ok()) {

            for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
                solver.calculateFk(links_fk[l_idx], col_model->links_[l_idx]->name, q);
            }

            publishJointState(q, joint_names);

            int m_id = 0;
            m_id = publishRobotModelVis(m_id, col_model, links_fk);

            loop_rate.sleep();
        }

    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_dynamic_model_cpp");
    TestDynamicModel test;
    test.spin();
    return 0;
}

