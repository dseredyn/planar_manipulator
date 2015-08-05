#include <ros/ros.h>
#include "ros/package.h"
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <stdlib.h>

#include "Eigen/Dense"
#include "Eigen/LU"

#include "planar5_dyn_model.h"
#include <self_collision/urdf_collision_parser.h>
#include "fk_ik.h"
#include "marker_publisher.h"

#ifdef EIGEN_RUNTIME_NO_MALLOC
#define RESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(false)
#define UNRESTRICT_ALLOC Eigen::internal::set_is_malloc_allowed(true)
#else
#define RESTRICT_ALLOC
#define UNRESTRICT_ALLOC
#endif

class TestDynamicModel {
    ros::NodeHandle nh_;
    ros::Publisher joint_state_pub_;
    ros::Publisher markers_pub_;
    tf::TransformBroadcaster br;

    const double PI;

public:
    TestDynamicModel() :
        PI(3.141592653589793)
    {
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
        markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/velma_markers", 10);
    }

    ~TestDynamicModel() {
    }

    double randomUniform(double min, double max) {
        return min + (max - min) * (double)rand() / (double)RAND_MAX;
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

    void publishTransform(const KDL::Frame &T_B_F, const std::string &frame_id) {
//        wrist_pose = pm.toMsg(T_B_F)
//        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), frame_id, "base")
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(T_B_F.p.x(), T_B_F.p.y(), T_B_F.p.z()) );
        tf::Quaternion q;
        double qx, qy, qz, qw;
        T_B_F.M.GetQuaternion(q[0], q[1], q[2], q[3]);
//        q.x = qx;
//        q.x = qx;
//        q.x = qx;
//        q.x = qx;
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", frame_id));
    }

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

        // initialize random seed
        srand(time(NULL));

        // dynamics model
        DynModelPlanar5 dyn_model;

        std::string robot_description_str;
        std::string robot_semantic_description_str;
        nh_.getParam("/robot_description", robot_description_str);
        nh_.getParam("/robot_semantic_description", robot_semantic_description_str);

        // collision model
        boost::shared_ptr<self_collision::CollisionModel> col_model = self_collision::CollisionModel::parseURDF(robot_description_str);
	    col_model-> parseSRDF(robot_semantic_description_str);

        std::vector<std::string > joint_names;
        joint_names.push_back("0_joint");
        joint_names.push_back("1_joint");
        joint_names.push_back("2_joint");
        joint_names.push_back("3_joint");
        joint_names.push_back("4_joint");

        std::string effector_name = "effector";
        int effector_idx = 0;
        for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
            if (col_model->links_[l_idx]->name == effector_name) {
                effector_idx = l_idx;
                break;
            }
        }

        int ndof = joint_names.size();

        // kinematics model
        FkIkSolver solver(robot_description_str);//joint_names, [], None);

        FkIkSolver::Jacobian J_r_HAND_6, J_r_HAND;
        J_r_HAND_6.resize(6, ndof);
        J_r_HAND.resize(3, ndof);

        Eigen::VectorXd q, dq, ddq, torque;
        q.resize( ndof );
        dq.resize( ndof );
        ddq.resize( ndof );
        torque.resize( ndof );
        for (int q_idx = 0; q_idx < ndof; q_idx++) {
            q[q_idx] = 0.1;
            dq[q_idx] = 0.0;
            ddq[q_idx] = 0.0;
            torque[q_idx] = 0.0;
        }

        dyn_model.accel(ddq, q, dq, torque);

        std::vector<KDL::Frame > links_fk(col_model->link_count_);

        // temporary variables
        Eigen::MatrixXd tmpNK_(ndof, 6);
        Eigen::MatrixXd tmpKK_(6, 6);
        Eigen::MatrixXd tmpKK2_(6, 6);
        Eigen::VectorXd tmpK_(6);
        Eigen::MatrixXd tmpNN_(ndof, ndof);
        Eigen::MatrixXd tmpKN_(6, ndof);
        Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> es_;
        Eigen::PartialPivLU<Eigen::MatrixXd> lu_, luKK_;

        Eigen::MatrixXd A(6, 6), Q(6, 6), Dc(6, 6);
        Eigen::VectorXd K0(6);

        // loop variables
        ros::Time last_time = ros::Time::now();
        KDL::Frame r_HAND_target;
        int loop_counter = 10000;
        ros::Rate loop_rate(1000);
        while (ros::ok()) {

            if (loop_counter > 800) {
                r_HAND_target = KDL::Frame(KDL::Rotation::RotZ(randomUniform(-PI, PI)), KDL::Vector(randomUniform(0,2), randomUniform(-1,1), 0));

                publishTransform(r_HAND_target, "effector_dest");
/*                qt = r_HAND_target.M.GetQuaternion()
                pt = r_HAND_target.p
                print "**** STATE ****"
                print "r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s), PyKDL.Vector(%s,%s,%s))"%(qt[0],qt[1],qt[2],qt[3],pt[0],pt[1],pt[2])
                print "self.q = np.array(", self.q, ")"
                print "self.dq = np.array(", self.dq, ")"
*/
                loop_counter = 0;
            }
            loop_counter += 1;

            // calculate forward kinematics for all links
            for (int l_idx = 0; l_idx < col_model->link_count_; l_idx++) {
                solver.calculateFk(links_fk[l_idx], col_model->links_[l_idx]->name, q);
            }

            //
            // effector task
            //
            KDL::Frame T_B_E = links_fk[effector_idx];
            KDL::Frame r_HAND_current = T_B_E;
            KDL::Twist diff = KDL::diff(r_HAND_current, r_HAND_target, 1.0);
            Eigen::VectorXd r_HAND_diff(3);
            r_HAND_diff[0] = diff[0];
            r_HAND_diff[1] = diff[1];
            r_HAND_diff[2] = diff[5];

            Eigen::VectorXd Kc(3);
            Kc[0] = 10.0;
            Kc[1] = 10.0;
            Kc[2] = 1.0;
            Eigen::VectorXd Dxi(3);
            Dxi[0] = 0.7;
            Dxi[1] = 0.7;
            Dxi[2] = 0.7;
            Eigen::VectorXd wrench(3);
            for (int dim_idx = 0; dim_idx < 3; dim_idx++) {
                wrench[dim_idx] = Kc[dim_idx] * r_HAND_diff[dim_idx];
            }

            solver.getJacobian(J_r_HAND_6, effector_name, q);

            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                J_r_HAND(0, q_idx) = J_r_HAND_6(0, q_idx);
                J_r_HAND(1, q_idx) = J_r_HAND_6(1, q_idx);
                J_r_HAND(2, q_idx) = J_r_HAND_6(5, q_idx);
            }
            Eigen::VectorXd torque_HAND = J_r_HAND.transpose() * wrench;

            // code form cartesian_impedance.h
            FkIkSolver::Jacobian JT = J_r_HAND.transpose();
            tmpNK_.noalias() = J_r_HAND * dyn_model.invI;
            A.noalias() = tmpNK_ * JT;
            luKK_.compute(A);
            A = luKK_.inverse();

            tmpKK_ = Kc.asDiagonal();
            UNRESTRICT_ALLOC;
            es_.compute(tmpKK_, A);
            RESTRICT_ALLOC;
            K0 = es_.eigenvalues();
            luKK_.compute(es_.eigenvectors());
            Q = luKK_.inverse();

            tmpKK_ = Dxi.asDiagonal();
            Dc.noalias() = Q.transpose() * tmpKK_;
            tmpKK_ = K0.cwiseSqrt().asDiagonal();
            tmpKK2_.noalias() = Dc *  tmpKK_;
            Dc.noalias() = tmpKK2_ * Q;
            tmpK_.noalias() = J_r_HAND * dq;
            wrench.noalias() = Dc * tmpK_;
            torque_HAND.noalias() -= JT * wrench;

            torque = torque_HAND;

            // simulate one step
            dyn_model.accel(ddq, q, dq, torque);
            float time_d = 0.01;
            for (int q_idx = 0; q_idx < ndof; q_idx++) {
                dq[q_idx] += ddq[q_idx] * time_d;
                q[q_idx] += dq[q_idx] * time_d;
            }

            // publish markers and robot state with limited rate
            ros::Duration time_elapsed = ros::Time::now() - last_time;
            if (time_elapsed.toSec() > 0.05) {
                publishJointState(q, joint_names);
                int m_id = 0;
                m_id = publishRobotModelVis(m_id, col_model, links_fk);
                ros::Time last_time = ros::Time::now();
            }
            ros::spinOnce();
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

