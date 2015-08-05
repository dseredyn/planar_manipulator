#!/usr/bin/env python

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

import roslib
roslib.load_manifest('planar_manipulator')

import rospy

from std_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *

import tf
from tf import *
from tf.transformations import * 
import tf_conversions.posemath as pm
from tf2_msgs.msg import *

import PyKDL
import math
import numpy as np
import copy
import random
import itertools
import rospkg

import velmautils
import fk_ik
import collision_model

class TestHierarchyControl:
    """

"""

    def __init__(self):
        self.pub_marker = velmautils.MarkerPublisher()
        self.pub_js = rospy.Publisher("/joint_states", sensor_msgs.msg.JointState, queue_size=100)
        self.br = tf.TransformBroadcaster()

    # returns (distance, p_pt1, p_pt2)
    def distancePoints(self, pt1, pt2):
        return (pt1-pt2).Norm(), pt1, pt2

    # returns (distance, p_line, p_pt)
    def distanceLinePoint(self, line, pt):
        a, b = line
        v = b - a
        ta = PyKDL.dot(v, a)
        tb = PyKDL.dot(v, b)
        tpt = PyKDL.dot(v, pt)
        if tpt <= ta:
            return (a-pt).Norm(), a, pt
        elif tpt >= tb:
            return (b-pt).Norm(), b, pt
        else:
            n = PyKDL.Vector(v.y(), -v.x(), v.z())
            n.Normalize()
            diff = PyKDL.dot(n, a) - PyKDL.dot(n, pt)
            return abs(diff), pt + (diff * n), pt

    # returns (distance, p_pt, p_line)
    def distancePointLine(self, pt, line):
        ret = self.distanceLinePoint(line, pt)
        return ret[0], ret[2], ret[1]

    # returns (distance, p_l1, p_l2)
    def distanceLines(self, l1, l2):
        x1, x2, x3, x4 = l1[0].x(), l1[1].x(), l2[0].x(), l2[1].x()
        y1, y2, y3, y4 = l1[0].y(), l1[1].y(), l2[0].y(), l2[1].y()
        denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
        if denom != 0.0:
            xi = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / denom
            yi = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / denom
            if ((xi >= x1 and xi <= x2) or (xi >= x2 and xi <= x1)) and\
                ((yi >= y1 and yi <= y2) or (yi >= y2 and yi <= y1)) and\
                ((xi >= x3 and xi <= x4) or (xi >= x4 and xi <= x3)) and\
                ((yi >= y3 and yi <= y4) or (yi >= y4 and yi <= y3)):
                return 0.0, PyKDL.Vector(xi, yi, 0.0), PyKDL.Vector(xi, yi, 0.0)
        dists = [
        (self.distanceLinePoint(l1, l2[0]), False),
        (self.distanceLinePoint(l1, l2[1]), False),
        (self.distanceLinePoint(l2, l1[0]), True),
        (self.distanceLinePoint(l2, l1[1]), True),
        ]
        min_dist = None
        for d, swap_points in dists:
            if min_dist == None or min_dist[0] > d[0]:
                if swap_points:
                    min_dist = (d[0], d[2], d[1])
                else:
                    min_dist = (d[0], d[1], d[2])
        return min_dist

    def publishJointState(self):
        js = sensor_msgs.msg.JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.joint_names
        for qi in self.q:
            js.position.append(qi)
        self.pub_js.publish(js)

    def publishTransform(self, T_B_F, frame_id):
        wrist_pose = pm.toMsg(T_B_F)
        self.br.sendTransform([wrist_pose.position.x, wrist_pose.position.y, wrist_pose.position.z], [wrist_pose.orientation.x, wrist_pose.orientation.y, wrist_pose.orientation.z, wrist_pose.orientation.w], rospy.Time.now(), frame_id, "base")

    def publishRobotModelVis(self, m_id, col_model, fk_map):
        namespace = "col_model"
        alpha = 0.5
        for link in col_model.links:
            if link.col == None:
                continue
            T_B_L = fk_map[link.name]
            for col in link.col:
                T_B_O = T_B_L * col.T_L_O
                if col.type == "sphere":
                    scale=Vector3(col.radius*2, col.radius*2, col.radius*2)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.SPHERE, scale=scale, T=T_B_O)
                elif col.type == "capsule":
                    m_id = self.pub_marker.publishFrameMarker(T_B_O, m_id, scale=0.1, frame='base', namespace=namespace)
                    scale=Vector3(col.radius*2, col.radius*2, col.radius*2)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,-col.length/2,0), m_id, r=0, g=1, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.SPHERE, scale=scale, T=T_B_O)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,col.length/2,0), m_id, r=0, g=1, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.SPHERE, scale=scale, T=T_B_O)
                    scale=Vector3(col.radius*2, col.radius*2, col.length)
                    T_O_C = PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi))
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=0, g=1, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.CYLINDER, scale=scale, T=T_B_O * T_O_C)
        return m_id

    def publishObstaclesVis(self, m_id, col_list):
        namespace = "col_model"
        alpha = 0.5
        for col in col_list:
                T_B_O = col.T_L_O
                if col.type == "sphere":
                    scale=Vector3(col.radius*2, col.radius*2, col.radius*2)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=0, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.SPHERE, scale=scale, T=T_B_O)
                elif col.type == "capsule":
                    m_id = self.pub_marker.publishFrameMarker(T_B_O, m_id, scale=0.1, frame='base', namespace=namespace)
                    scale=Vector3(col.radius*2, col.radius*2, col.radius*2)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,-col.length/2,0), m_id, r=1, g=0, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.SPHERE, scale=scale, T=T_B_O)
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(0,col.length/2,0), m_id, r=1, g=0, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.SPHERE, scale=scale, T=T_B_O)
                    scale=Vector3(col.radius*2, col.radius*2, col.length)
                    T_O_C = PyKDL.Frame(PyKDL.Rotation.RotX(90.0/180.0*math.pi))
                    m_id = self.pub_marker.publishSinglePointMarker(PyKDL.Vector(), m_id, r=1, g=0, b=0, a=alpha, namespace=namespace, frame_id='base', m_type=Marker.CYLINDER, scale=scale, T=T_B_O * T_O_C)
        return m_id

    def spin(self):

        rospack = rospkg.RosPack()

        model = "5dof"
#        model = "6dof"
#        model = "two_arms"

        if model == "6dof":
            urdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_manipulator_6dof.urdf'
            srdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_manipulator_6dof.srdf'
        elif model == "5dof":
            urdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_manipulator.urdf'
            srdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_manipulator.srdf'
        elif model == "two_arms":
            urdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_two_arms.urdf'
            srdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_two_arms.srdf'

        # TEST: line - line distance
        if False:
            while not rospy.is_shutdown():
                pt = PyKDL.Vector(random.uniform(-1, 1), random.uniform(-1, 1), 0)
                line = (PyKDL.Vector(random.uniform(-1, 1),random.uniform(-1, 1),0), PyKDL.Vector(random.uniform(-1, 1),random.uniform(-1, 1),0))
                line2 = (PyKDL.Vector(random.uniform(-1, 1),random.uniform(-1, 1),0), PyKDL.Vector(random.uniform(-1, 1),random.uniform(-1, 1),0))
                dist, p1, p2 = self.distanceLines(line, line2)

                m_id = 0
                m_id = self.pub_marker.publishVectorMarker(line[0], line[1], m_id, 0, 1, 0, frame='world', namespace='default', scale=0.02)
                m_id = self.pub_marker.publishVectorMarker(line2[0], line2[1], m_id, 1, 0, 0, frame='world', namespace='default', scale=0.02)
                m_id = self.pub_marker.publishVectorMarker(p1, p2, m_id, 1, 1, 1, frame='world', namespace='default', scale=0.02)
                print line, line2, dist
                raw_input(".")

            exit(0)

        # TEST: point - point distance
        if False:
            while not rospy.is_shutdown():
                pt1 = PyKDL.Vector(random.uniform(-1, 1), random.uniform(-1, 1), 0)
                pt2 = PyKDL.Vector(random.uniform(-1, 1), random.uniform(-1, 1), 0)
                dist, p1, p2 = self.distancePoints(pt1, pt2)

                m_id = 0
                m_id = self.pub_marker.publishSinglePointMarker(p1, m_id, r=0, g=1, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), T=None)
                m_id = self.pub_marker.publishSinglePointMarker(p2, m_id, r=1, g=0, b=0, a=1, namespace='default', frame_id='world', m_type=Marker.SPHERE, scale=Vector3(0.1, 0.1, 0.1), T=None)
                m_id = self.pub_marker.publishVectorMarker(p1, p2, m_id, 1, 0, 0, frame='world', namespace='default', scale=0.02)
                print pt1, pt2, dist
                raw_input(".")

            exit(0)

        col = collision_model.CollisionModel()
        col.readUrdfSrdf(urdf_path, srdf_path)

        if model == "6dof":
            self.joint_names = ["0_joint", "1_joint", "2_joint", "3_joint", "4_joint", "5_joint"]
            effector_name = 'effector'
        elif model == "5dof":
            self.joint_names = ["0_joint", "1_joint", "2_joint", "3_joint", "4_joint"]
            effector_name = 'effector'
        elif model == "two_arms":
            self.joint_names = ["torso_0_joint", "left_0_joint", "left_1_joint", "left_2_joint", "left_3_joint", "right_0_joint", "right_1_joint", "right_2_joint", "right_3_joint"]
            effector_name = 'left_effector'
        self.q = np.zeros( len(self.joint_names) )
        test_cases = [
        (1.1, -2.3, 1.5, 1.5, 1.5),
        (-1.1, 2.3, -1.5, -1.5, -1.5),
        (0.0, 0.0, 1.5, 1.5, 1.5),
        (0.0, 0.0, -1.5, -1.5, -1.5),
        (0.2, 0.4, 1.5, 1.5, 1.5),
        (-0.2, -0.4, -1.5, -1.5, -1.5),
        ]
#        self.q[0] = 1.1
#        self.q[1] = -2.3
#        self.q[2] = 1.5
#        self.q[3] = 1.5
#        self.q[4] = 1.5

#        self.q = np.array(test_cases[4])

        solver = fk_ik.FkIkSolver(self.joint_names, [], None)

        self.publishJointState()

#        rospy.sleep(1)
#        exit(0)

        obst = []
        obj = collision_model.CollisionModel.Collision()
        obj.type = "capsule"
        obj.T_L_O = PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi), PyKDL.Vector(1.5, 0.7, 0))
        obj.radius = 0.1
        obj.length = 0.4
        obst.append( obj )

        obj = collision_model.CollisionModel.Collision()
        obj.type = "capsule"
        obj.T_L_O = PyKDL.Frame(PyKDL.Rotation.RotZ(90.0/180.0*math.pi), PyKDL.Vector(1.2, -0.9, 0))
        obj.radius = 0.1
        obj.length = 0.4
        obst.append( obj )

        obj = collision_model.CollisionModel.Collision()
        obj.type = "sphere"
        obj.T_L_O = PyKDL.Frame(PyKDL.Vector(1, 0, 0))
        obj.radius = 0.05
        obst.append( obj )

#        rospy.sleep(1)
#        T_B_E = solver.calculateFk("base", "effector", self.q)
#        print T_B_E

        r_HAND_targets = [
        PyKDL.Frame(PyKDL.Vector(0.1,1.0,0.0)),
        PyKDL.Frame(PyKDL.Vector(0.1,1.7,0.0)),
#        PyKDL.Frame(PyKDL.Rotation.RotY(90.0/180.0*math.pi), PyKDL.Vector(0.2,-0.5,0.0)),
        ]

        target_idx = 0
        r_HAND_target = r_HAND_targets[target_idx]
        target_idx += 1

        r_HAND_target = PyKDL.Frame(PyKDL.Vector(0.5,0.5,0))
        last_time = rospy.Time.now()


        counter = 10000
        while not rospy.is_shutdown():
            if counter > 800:
                if model == "two_arms":
                    r_HAND_target = PyKDL.Frame(PyKDL.Rotation.RotZ(random.uniform(-math.pi, math.pi)), PyKDL.Vector(random.uniform(-1,0), random.uniform(0,1.8), 0))
                else:
                    r_HAND_target = PyKDL.Frame(PyKDL.Rotation.RotZ(random.uniform(-math.pi, math.pi)), PyKDL.Vector(random.uniform(0,2), random.uniform(-1,1), 0))

#                r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,-0.98100002989,0.194007580664), PyKDL.Vector(-0.129108034334,0.518606130706,0.0))

#                r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.470814280381,0.882232346601), PyKDL.Vector(0.676567476122,0.0206561816531,0.0))

#r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.50451570265,0.863402516663), PyKDL.Vector(0.252380653828,0.923309935287,0.0))
#self.q = np.array( [-0.29968745  0.66939973 -2.49850991  1.87533697  2.63546305] )
#r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.704294768084,0.70990765572), PyKDL.Vector(0.334245569765,1.82368612057,0.0))
#self.q = np.array( [ 0.33203731  0.071835   -2.46646112  1.14339024  1.97684146] )

#                r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.924467039084,-0.381261975087), PyKDL.Vector(0.261697539135,0.97235224304,0.0))
#r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(0.0,0.0,0.894763681298,0.446539980999), PyKDL.Vector(0.354981453046,0.604598917063,0.0))
#self.q = np.array( [-0.89640518  0.44336642  1.96125279 -1.66533209 -2.19189403] )

                qt = r_HAND_target.M.GetQuaternion()
                pt = r_HAND_target.p
                print "r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s), PyKDL.Vector(%s,%s,%s))"%(qt[0],qt[1],qt[2],qt[3],pt[0],pt[1],pt[2])
                print "self.q = np.array(", self.q, ")"
                counter = 0
            counter += 1

            time_elapsed = rospy.Time.now() - last_time

            J_JLC = np.matrix(numpy.zeros( (len(self.q), len(self.q)) ))
            delta_V_JLC = np.empty(len(self.q))
            for q_idx in range(len(self.q)):
                if self.q[q_idx] < solver.lim_lower_soft[q_idx]:
                    delta_V_JLC[q_idx] = self.q[q_idx] - solver.lim_lower_soft[q_idx]
                    J_JLC[q_idx,q_idx] = min(1.0, 10*abs(self.q[q_idx] - solver.lim_lower_soft[q_idx]) / abs(solver.lim_lower[q_idx] - solver.lim_lower_soft[q_idx]))
                elif self.q[q_idx] > solver.lim_upper_soft[q_idx]:
                    delta_V_JLC[q_idx] = self.q[q_idx] - solver.lim_upper_soft[q_idx]
                    J_JLC[q_idx,q_idx] = min(1.0, 10*abs(self.q[q_idx] - solver.lim_upper_soft[q_idx]) / abs(solver.lim_upper[q_idx] - solver.lim_upper_soft[q_idx]))
                else:
                    delta_V_JLC[q_idx] = 0.0
                    J_JLC[q_idx,q_idx] = 0.0

            J_JLC_inv = J_JLC.transpose()#np.linalg.pinv(J_JLC)

            N_JLC = np.matrix(np.identity(len(self.q))) - (J_JLC_inv * J_JLC)
            N_JLC_inv = np.linalg.pinv(N_JLC)

            v_max_JLC = 20.0/180.0*math.pi
            kp_JLC = 10.0
            dx_JLC_des = kp_JLC * delta_V_JLC

            # min(1.0, v_max_JLC/np.linalg.norm(dx_JLC_des))
            if v_max_JLC > np.linalg.norm(dx_JLC_des):
                 vv_JLC = 1.0
            else:
                vv_JLC = v_max_JLC/np.linalg.norm(dx_JLC_des)
            dx_JLC_ref = - vv_JLC * dx_JLC_des

            J_r_HAND = solver.getJacobian('base', effector_name, self.q, base_end=False)
            J_r_HAND_inv = np.linalg.pinv(J_r_HAND)
            T_B_E = solver.calculateFk('base', effector_name, self.q)
            r_HAND_current = T_B_E
            r_HAND_diff = PyKDL.diff(r_HAND_current, r_HAND_target)

            delta_V_HAND = np.empty(6)
            delta_V_HAND[0] = r_HAND_diff.vel[0]
            delta_V_HAND[1] = r_HAND_diff.vel[1]
            delta_V_HAND[2] = r_HAND_diff.vel[2]
            delta_V_HAND[3] = r_HAND_diff.rot[0]
            delta_V_HAND[4] = r_HAND_diff.rot[1]
            delta_V_HAND[5] = r_HAND_diff.rot[2]

            v_max_HAND = 4.0
            kp_HAND = 10.0
            dx_HAND_des = kp_HAND * delta_V_HAND
            if v_max_HAND > np.linalg.norm(dx_HAND_des):
                vv_HAND = 1.0
            else:
                vv_HAND = v_max_HAND/np.linalg.norm(dx_HAND_des)
            dx_r_HAND_ref = vv_HAND * dx_HAND_des

            links_fk = {}
            for link in col.links:
                links_fk[link.name] = solver.calculateFk('base', link.name, self.q)

            activation_dist = 0.05

            link_collision_map = {}
            if True:
                # self collision
                total_contacts = 0
                for link1_name, link2_name in col.collision_pairs:

                    link1 = col.link_map[link1_name]
                    T_B_L1 = links_fk[link1_name]
                    link2 = col.link_map[link2_name]
                    T_B_L2 = links_fk[link2_name]

                    for col1 in link1.col:
                        for col2 in link2.col:
                            T_B_C1 = T_B_L1 * col1.T_L_O
                            T_B_C2 = T_B_L2 * col2.T_L_O
                            
                            c_dist = (T_B_C1 * PyKDL.Vector() - T_B_C2 * PyKDL.Vector()).Norm()
                            if col1.type == "capsule":
                                c_dist -= col1.radius + col1.length/2
                            elif col1.type == "sphere":
                                c_dist -= col1.radius
                            if col2.type == "capsule":
                                c_dist -= col2.radius + col2.length/2
                            elif col2.type == "sphere":
                                c_dist -= col2.radius
                            if c_dist > activation_dist:
                                continue

                            dist = None
                            if col1.type == "capsule" and col2.type == "capsule":
                                line1 = (T_B_C1 * PyKDL.Vector(0, -col1.length/2, 0), T_B_C1 * PyKDL.Vector(0, col1.length/2, 0))
                                line2 = (T_B_C2 * PyKDL.Vector(0, -col2.length/2, 0), T_B_C2 * PyKDL.Vector(0, col2.length/2, 0))
                                dist, p1_B, p2_B = self.distanceLines(line1, line2)
                            elif col1.type == "capsule" and col2.type == "sphere":
                                line = (T_B_C1 * PyKDL.Vector(0, -col1.length/2, 0), T_B_C1 * PyKDL.Vector(0, col1.length/2, 0))
                                pt = T_B_C2 * PyKDL.Vector()
                                dist, p1_B, p2_B = self.distanceLinePoint(line, pt)
                            elif col1.type == "sphere" and col2.type == "capsule":
                                pt = T_B_C1 * PyKDL.Vector()
                                line = (T_B_C2 * PyKDL.Vector(0, -col2.length/2, 0), T_B_C2 * PyKDL.Vector(0, col2.length/2, 0))
                                dist, p1_B, p2_B = self.distancePointLine(pt, line)
                            elif col1.type == "sphere" and col2.type == "sphere":
                                dist, p1_B, p2_B = self.distancePoints(T_B_C1 * PyKDL.Vector(), T_B_C2 * PyKDL.Vector())
                            else:
                                print "ERROR: unknown collision type:", col1.type, col2.type
                                exit(0)

                            if dist != None:
#                                print "dist", dist, link1_name, link2_name, col1.type, col2.type
                                dist -= col1.radius + col2.radius
                                v = p2_B - p1_B
                                v.Normalize()
                                n1_B = v
                                n2_B = -v
                                p1_B += n1_B * col1.radius
                                p2_B += n2_B * col2.radius

                                if dist < activation_dist:
                                    if not (link1_name, link2_name) in link_collision_map:
                                        link_collision_map[(link1_name, link2_name)] = []
                                    link_collision_map[(link1_name, link2_name)].append( (p1_B, p2_B, dist, n1_B, n2_B) )

                # environment collisions
                for link in col.links:
                    if link.col == None:
                        continue
                    link1_name = link.name
                    T_B_L1 = links_fk[link1_name]
                    T_B_L2 = links_fk["base"]
                    for col1 in link.col:
                        for col2 in obst:
                            T_B_C1 = T_B_L1 * col1.T_L_O
                            T_B_C2 = T_B_L2 * col2.T_L_O
                            dist = None
                            if col1.type == "capsule" and col2.type == "capsule":
                                line1 = (T_B_C1 * PyKDL.Vector(0, -col1.length/2, 0), T_B_C1 * PyKDL.Vector(0, col1.length/2, 0))
                                line2 = (T_B_C2 * PyKDL.Vector(0, -col2.length/2, 0), T_B_C2 * PyKDL.Vector(0, col2.length/2, 0))
                                dist, p1_B, p2_B = self.distanceLines(line1, line2)
                            elif col1.type == "capsule" and col2.type == "sphere":
                                line = (T_B_C1 * PyKDL.Vector(0, -col1.length/2, 0), T_B_C1 * PyKDL.Vector(0, col1.length/2, 0))
                                pt = T_B_C2 * PyKDL.Vector()
                                dist, p1_B, p2_B = self.distanceLinePoint(line, pt)
                            elif col1.type == "sphere" and col2.type == "capsule":
                                pt = T_B_C1 * PyKDL.Vector()
                                line = (T_B_C2 * PyKDL.Vector(0, -col2.length/2, 0), T_B_C2 * PyKDL.Vector(0, col2.length/2, 0))
                                dist, p1_B, p2_B = self.distancePointLine(pt, line)
                            elif col1.type == "sphere" and col2.type == "sphere":
                                dist, p1_B, p2_B = self.distancePoints(T_B_C1 * PyKDL.Vector(), T_B_C2 * PyKDL.Vector())
#                                print "a:",dist, p1_B, p2_B
                            else:
                                print "ERROR: unknown collision type:", col1.type, col2.type
                                exit(0)

                            if dist != None:
                                dist -= col1.radius + col2.radius
                                v = p2_B - p1_B
                                v.Normalize()
                                n1_B = v
                                n2_B = -v
                                p1_B += n1_B * col1.radius
                                p2_B += n2_B * col2.radius

#                                if col1.type == "sphere" and col2.type == "sphere":
#                                    print "b:",dist, p1_B, p2_B

                                if dist < activation_dist:
                                    if not (link1_name, "base") in link_collision_map:
                                        link_collision_map[(link1_name, "base")] = []
                                    link_collision_map[(link1_name, "base")].append( (p1_B, p2_B, dist, n1_B, n2_B) )
                            

            omega_col = np.matrix(np.zeros( (len(self.q),1) ))
            Ncol = np.matrix(np.identity(len(self.q)))
            m_id = 0
            for link1_name, link2_name in link_collision_map:
                for (p1_B, p2_B, dist, n1_B, n2_B) in link_collision_map[ (link1_name, link2_name) ]:
                    if dist > 0.0:
                        m_id = self.pub_marker.publishVectorMarker(p1_B, p2_B, m_id, 1, 1, 1, frame='base', namespace='default', scale=0.02)
                    else:
                        m_id = self.pub_marker.publishVectorMarker(p1_B, p2_B, m_id, 1, 0, 0, frame='base', namespace='default', scale=0.02)

                    T_B_L1 = links_fk[link1_name]
                    T_L1_B = T_B_L1.Inverse()
                    T_B_L2 = links_fk[link2_name]
                    T_L2_B = T_B_L2.Inverse()
                    p1_L1 = T_L1_B * p1_B
                    p2_L2 = T_L2_B * p2_B
                    n1_L1 = PyKDL.Frame(T_L1_B.M) * n1_B
                    n2_L2 = PyKDL.Frame(T_L2_B.M) * n2_B

#                    print p1_L1, p1_L1+n1_L1*0.2
#                    print p2_L2, p2_L2+n2_L2*0.2
#                    m_id = self.pub_marker.publishVectorMarker(p1_L1, p1_L1+n1_L1*0.2, m_id, 0, 1, 0, frame=link1_name, namespace='default', scale=0.02)
#                    m_id = self.pub_marker.publishVectorMarker(T_B_L2*p2_L2, T_B_L2*(p2_L2+n2_L2*0.2), m_id, 0, 0, 1, frame="base", namespace='default', scale=0.02)

                    jac1 = PyKDL.Jacobian(len(self.q))
                    jac2 = PyKDL.Jacobian(len(self.q))
                    common_link_name = solver.getJacobiansForPairX(jac1, jac2, link1_name, p1_L1, link2_name, p2_L2, self.q, None)

#                    T_B_Lc = links_fk[common_link_name]

#                    jac1.changeBase(T_B_Lc.M)
#                    jac2.changeBase(T_B_Lc.M)
#                    T_Lc_L1 = T_B_Lc.Inverse() * T_B_L1
#                    T_Lc_L2 = T_B_Lc.Inverse() * T_B_L2
#                    n1_Lc = PyKDL.Frame(T_Lc_L1.M) * n1_L1
#                    n2_Lc = PyKDL.Frame(T_Lc_L2.M) * n2_L2

#                    print link1_name, link2_name
#                    print "jac1"
#                    print jac1
#                    print "jac2"
#                    print jac2

                    # repulsive velocity
                    V_max = 20.0
                    dist = max(dist, 0.0)
                    depth = (activation_dist - dist)
#                    Vrep = V_max * depth * depth / (activation_dist * activation_dist)
                    Vrep = V_max * depth / activation_dist
#                    Vrep = -max(Vrep, 0.01)
                    Vrep = -Vrep

#                    # the mapping between motions along contact normal and the Cartesian coordinates
                    e1 = n1_L1
                    e2 = n2_L2
                    Jd1 = np.matrix([e1[0], e1[1], e1[2]])
                    Jd2 = np.matrix([e2[0], e2[1], e2[2]])

                    # rewrite the linear part of the jacobian
                    jac1_mx = np.matrix(np.zeros( (3, len(self.q)) ))
                    jac2_mx = np.matrix(np.zeros( (3, len(self.q)) ))
                    for q_idx in range(len(self.q)):
                        col1 = jac1.getColumn(q_idx)
                        col2 = jac2.getColumn(q_idx)
                        for row_idx in range(3):
                            jac1_mx[row_idx, q_idx] = col1[row_idx]
                            jac2_mx[row_idx, q_idx] = col2[row_idx]

                    Jcol1 = Jd1 * jac1_mx
                    Jcol2 = Jd2 * jac2_mx

                    Jcol = np.matrix(np.zeros( (2, len(self.q)) ))
                    for q_idx in range(len(self.q)):
                        Jcol[0, q_idx] = Jcol1[0, q_idx]
                        Jcol[1, q_idx] = Jcol2[0, q_idx]

#                    Jcol = Jcol * (Ncol * N_JLC)
                    # TODO: is the transposition ok?
                    Jcol_pinv = np.linalg.pinv(Jcol)
#                    Jcol_pinv = Jcol.transpose()

                    activation = min(1.0, 2.0*depth/activation_dist)
                    a_des = np.matrix(np.zeros( (len(self.q),len(self.q)) ))
                    a_des[0,0] = a_des[1,1] = activation

                    U, S, V = numpy.linalg.svd(Jcol, full_matrices=True, compute_uv=True)

#                    print "activation", activation
#                    print "Jcol"
#                    print Jcol
#                    raw_input(".")
                    if rospy.is_shutdown():
                        exit(0)

#                    print "V"
#                    print V
#                    print "S"
#                    print S

#                    Ncol12 = np.matrix(np.identity(len(self.q))) - Jcol.transpose() * (Jcol_pinv).transpose()
                    Ncol12 = np.matrix(np.identity(len(self.q))) - (V * a_des * V.transpose())
                    Ncol = Ncol * Ncol12
#                    d_omega = Jcol_prec_inv * np.matrix([Vrep, Vrep]).transpose()

                    d_omega = Jcol_pinv * np.matrix([Vrep, Vrep]).transpose()
                    omega_col += d_omega

#            print "omega_col", omega_col
#            print dx_HAND_ref

#            omega_r_HAND = (J_r_HAND_inv * np.matrix(dx_r_HAND_ref).transpose())

            self.pub_marker.eraseMarkers(m_id, 10, frame_id='base', namespace='default')


            Ncol_inv = np.linalg.pinv(Ncol)

            J_r_HAND_prec = J_r_HAND * (Ncol * N_JLC)
            J_r_HAND_prec_inv = np.linalg.pinv(J_r_HAND_prec)

            omega = J_JLC_inv * np.matrix(dx_JLC_ref).transpose() + N_JLC.transpose() * (omega_col + (Ncol.transpose() * J_r_HAND_inv) * np.matrix(dx_r_HAND_ref).transpose())
#            omega = J_JLC_inv * np.matrix(dx_JLC_ref).transpose() + N_JLC_inv         * (omega_col + (Ncol_inv * J_r_HAND_prec_inv) * np.matrix(dx_r_HAND_ref).transpose())
#            omega = J_JLC_inv * np.matrix(dx_JLC_ref).transpose() + np.linalg.pinv(N_JLC) * omega_col
#            omega = omega_col

            omega_vector = np.empty(len(self.q))
            for q_idx in range(len(self.q)):
                omega_vector[q_idx] = omega[q_idx][0]

            max_norm = 0.5
            omega_norm = np.linalg.norm(omega_vector)
            if omega_norm > max_norm:
                omega_vector *= max_norm / omega_norm

            self.q += omega_vector * 0.02

            
            if time_elapsed.to_sec() > 0.05:
#                print self.q
                m_id = 0
                m_id = self.publishRobotModelVis(m_id, col, links_fk)
                m_id = self.publishObstaclesVis(m_id, obst)
                last_time = rospy.Time.now()
                self.publishJointState()
                self.publishTransform(r_HAND_target, "hand_dest")

#            rospy.sleep(0.01)



if __name__ == '__main__':

    rospy.init_node('test_hierarchy_control')

    task = TestHierarchyControl()
    rospy.sleep(0.5)

    task.spin()


