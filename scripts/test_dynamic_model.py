#!/usr/bin/env python

# Copyright (c) 2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology
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
import scipy
import copy
import random
import itertools
import rospkg

import velmautils
import fk_ik
import collision_model
import planar5

class TestDynamicModel:
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

    def jointLimitTrq(self, hl, ll, ls, r_max, q):
        if q > (hl - ls):
            return -1.0 * ((q - hl + ls) / ls) * ((q - hl + ls) / ls) * r_max
        elif q < (ll + ls):
            return ((ll + ls - q) / ls) * ((ll + ls - q) / ls) * r_max
        else:
            return 0.0

    def spin(self):

        rospack = rospkg.RosPack()

        urdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_manipulator.urdf'
        srdf_path=rospack.get_path('planar_manipulator_defs') + '/robots/planar_manipulator.srdf'

        dyn_model = planar5.DynModelPlanar5()

        col = collision_model.CollisionModel()
        col.readUrdfSrdf(urdf_path, srdf_path)

        self.joint_names = ["0_joint", "1_joint", "2_joint", "3_joint", "4_joint"]
        effector_name = 'effector'

        ndof = len(self.joint_names)
        # robot state
        self.q = np.zeros( ndof )
        self.dq = np.zeros( ndof )
        self.q[0] = 0.2
        self.q[1] = -0.1
        self.q[2] = -0.1
        self.q[3] = -0.1
        self.q[4] = -0.1

        solver = fk_ik.FkIkSolver(self.joint_names, [], None)

        self.publishJointState()

        # obstacles
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
        obj.T_L_O = PyKDL.Frame(PyKDL.Vector(1, -0.2, 0))
        obj.radius = 0.05
        obst.append( obj )

        last_time = rospy.Time.now()

        limit_range = np.zeros( ndof )
        max_trq = np.zeros( ndof )
        for q_idx in range( ndof ):
            limit_range[q_idx] = 15.0/180.0*math.pi
            max_trq[q_idx] = 10.0

        dyn_model.computeM(self.q)

        obst_offset = 0.0
        counter = 10000
        while not rospy.is_shutdown():
            obst_offset += 0.0001
            obst[-1].T_L_O = PyKDL.Frame(PyKDL.Vector(1, -0.2+math.sin(obst_offset), 0))

            if counter > 800:
                r_HAND_target = PyKDL.Frame(PyKDL.Rotation.RotZ(random.uniform(-math.pi, math.pi)), PyKDL.Vector(random.uniform(0,2), random.uniform(-1,1), 0))

                qt = r_HAND_target.M.GetQuaternion()
                pt = r_HAND_target.p
                print "**** STATE ****"
                print "r_HAND_target = PyKDL.Frame(PyKDL.Rotation.Quaternion(%s,%s,%s,%s), PyKDL.Vector(%s,%s,%s))"%(qt[0],qt[1],qt[2],qt[3],pt[0],pt[1],pt[2])
                print "self.q = np.array(", self.q, ")"
                print "self.dq = np.array(", self.dq, ")"
                counter = 0
            counter += 1

            time_elapsed = rospy.Time.now() - last_time

            #
            # mass matrix
            #
            dyn_model.computeM(self.q)
            M = dyn_model.M
            Minv = dyn_model.Minv

            #
            # JLC
            #
            torque_JLC = np.zeros( ndof )
            K = np.zeros( (ndof, ndof) )
            for q_idx in range( ndof ):
                torque_JLC[q_idx] = self.jointLimitTrq(solver.lim_upper[q_idx], solver.lim_lower[q_idx], limit_range[q_idx], max_trq[q_idx], self.q[q_idx])
                if abs(torque_JLC[q_idx]) > 0.001:
                    K[q_idx,q_idx] = max_trq[q_idx]/limit_range[q_idx]
                else:
                    K[q_idx,q_idx] = 0.001

            w, v = scipy.linalg.eigh(a=K, b=M)
#            q_ = dyn_model.gaussjordan(np.matrix(v))
            q_ = np.linalg.inv( np.matrix(v) )
            k0_ = w

            k0_sqrt = np.zeros( k0_.shape )
            for q_idx in range( ndof ):
                k0_sqrt[q_idx] = math.sqrt(k0_[q_idx])
            tmpNN_ = np.diag(k0_sqrt)

            D = 2.0 * q_.H * 0.7 * tmpNN_ * q_

            torque_JLC_mx = -D * np.matrix(self.dq).transpose()
            for q_idx in range( ndof ):
                torque_JLC[q_idx] += torque_JLC_mx[q_idx, 0]

            # calculate jacobian (the activation function)
            J_JLC = np.matrix(numpy.zeros( (ndof, ndof) ))
            for q_idx in range( ndof ):
                if self.q[q_idx] < solver.lim_lower_soft[q_idx]:
                    J_JLC[q_idx,q_idx] = min(1.0, 10*abs(self.q[q_idx] - solver.lim_lower_soft[q_idx]) / abs(solver.lim_lower[q_idx] - solver.lim_lower_soft[q_idx]))
                elif self.q[q_idx] > solver.lim_upper_soft[q_idx]:
                    J_JLC[q_idx,q_idx] = min(1.0, 10*abs(self.q[q_idx] - solver.lim_upper_soft[q_idx]) / abs(solver.lim_upper[q_idx] - solver.lim_upper_soft[q_idx]))
                else:
                    J_JLC[q_idx,q_idx] = 0.0
            N_JLC = np.matrix(np.identity( ndof )) - (J_JLC.transpose() * J_JLC)

            links_fk = {}
            for link in col.links:
                links_fk[link.name] = solver.calculateFk('base', link.name, self.q)

            #
            # collision constraints
            #
            link_collision_map = {}
            if True:
                activation_dist = 0.05
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

                                if dist < activation_dist:
                                    if not (link1_name, "base") in link_collision_map:
                                        link_collision_map[(link1_name, "base")] = []
                                    link_collision_map[(link1_name, "base")].append( (p1_B, p2_B, dist, n1_B, n2_B) )
                            

            torque_col = np.matrix(np.zeros( (ndof,1) ))
            Ncol = np.matrix(np.identity( ndof ))
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

                    jac1 = PyKDL.Jacobian( ndof )
                    jac2 = PyKDL.Jacobian( ndof )
                    common_link_name = solver.getJacobiansForPairX(jac1, jac2, link1_name, p1_L1, link2_name, p2_L2, self.q, None)

#                    print link1_name, link2_name

                    depth = (activation_dist - dist)

                    # repulsive force
                    Fmax = 20.0
                    if dist <= activation_dist:
                        f = (dist - activation_dist) / activation_dist
                    else:
                        f = 0.0
                    Frep = Fmax * f * f

                    K = 2.0 * Fmax / (activation_dist * activation_dist)

                    # the mapping between motions along contact normal and the Cartesian coordinates
                    e1 = n1_L1
                    e2 = n2_L2
                    Jd1 = np.matrix([e1[0], e1[1], e1[2]])
                    Jd2 = np.matrix([e2[0], e2[1], e2[2]])

                    # rewrite the linear part of the jacobian
                    jac1_mx = np.matrix(np.zeros( (3, ndof) ))
                    jac2_mx = np.matrix(np.zeros( (3, ndof) ))
                    for q_idx in range(ndof):
                        col1 = jac1.getColumn(q_idx)
                        col2 = jac2.getColumn(q_idx)
                        for row_idx in range(3):
                            jac1_mx[row_idx, q_idx] = col1[row_idx]
                            jac2_mx[row_idx, q_idx] = col2[row_idx]

                    Jcol1 = Jd1 * jac1_mx
                    Jcol2 = Jd2 * jac2_mx

                    Jcol = np.matrix(np.zeros( (1, ndof) ))
                    for q_idx in range(ndof):
                        Jcol[0, q_idx] = Jcol1[0, q_idx] + Jcol2[0, q_idx]

                    # calculate relative velocity between points
                    ddij = Jcol * np.matrix(self.dq).transpose()

                    activation = min(1.0, 5.0*depth/activation_dist)
                    activation = max(0.0, activation)
                    if ddij <= 0.0:
                        activation = 0.0
                    a_des = activation

#                    print "activation", activation
#                    raw_input(".")
                    if rospy.is_shutdown():
                        exit(0)

                    Ncol12 = np.matrix(np.identity(ndof)) - (Jcol.transpose() * a_des * Jcol)
                    Ncol = Ncol * Ncol12

                    # calculate collision mass
                    Mdij = Jcol * Minv * Jcol.transpose()

                    D = 2.0 * 0.7 * math.sqrt(Mdij * K)
                    d_torque = Jcol.transpose() * (-Frep - D * ddij)
                    torque_col += d_torque

            self.pub_marker.eraseMarkers(m_id, 10, frame_id='base', namespace='default')

            #
            # end-effector task
            #
            T_B_E = links_fk[effector_name]
            r_HAND_current = T_B_E
            diff = PyKDL.diff(r_HAND_current, r_HAND_target)
            r_HAND_diff = np.array( [diff[0], diff[1], diff[5]] )

            Kc = np.array( [10, 10, 1] )
            Dxi = np.array( [0.7, 0.7, 0.7] )
            wrench = np.zeros( 3 )
            for dim in range(3):
                wrench[dim] = Kc[dim] * r_HAND_diff[dim]

            J_r_HAND_6 = solver.getJacobian('base', effector_name, self.q)

            J_r_HAND = np.matrix( np.zeros( (3,5) ) )
            for q_idx in range( ndof ):
                J_r_HAND[0, q_idx] = J_r_HAND_6[0, q_idx]
                J_r_HAND[1, q_idx] = J_r_HAND_6[1, q_idx]
                J_r_HAND[2, q_idx] = J_r_HAND_6[5, q_idx]

            torque_HAND = J_r_HAND.transpose() * np.matrix(wrench).transpose()

            A = J_r_HAND * Minv * J_r_HAND.transpose()

#            A = dyn_model.gaussjordan(A)
            A = np.linalg.inv(A)

            tmpKK_ = np.matrix(np.diag(Kc))

            w, v = scipy.linalg.eigh(a=tmpKK_, b=A)
#            Q = dyn_model.gaussjordan(np.matrix(v))
            Q = np.linalg.inv( np.matrix(v) )
            K0 = w

            Dc = Q.transpose() * np.matrix( np.diag(Dxi) )

            K0_sqrt = np.zeros( K0.shape )
            for dim_idx in range( 3 ):
                K0_sqrt[dim_idx] = math.sqrt(K0[dim_idx])
            Dc = 2.0 * Dc *  np.matrix( np.diag(K0_sqrt) ) * Q

            F = Dc * J_r_HAND * np.matrix(self.dq).transpose()
            torque_HAND_mx = -J_r_HAND.transpose() * F
            for q_idx in range( ndof ):
                torque_HAND[q_idx] += torque_HAND_mx[q_idx, 0]

            # null-space
#    tmpNK_.noalias() = J * Mi;
#    tmpKK_.noalias() = tmpNK_ * JT;
#    luKK_.compute(tmpKK_);
#    tmpKK_ = luKK_.inverse();
#    tmpKN_.noalias() = Mi * JT;
#    Ji.noalias() = tmpKN_ * tmpKK_;
#
#    P.noalias() = Eigen::MatrixXd::Identity(P.rows(), P.cols());
#    P.noalias() -=  J.transpose() * A * J * Mi;

            torque = np.matrix(torque_JLC).transpose() + N_JLC.transpose() * (torque_col + (Ncol.transpose() * torque_HAND))
#            torque = torque_HAND

            time_d = 0.01
            # simulate one step
            ddq = dyn_model.accel(self.q, self.dq, torque)
            for q_idx in range(ndof):
                self.dq[q_idx] += ddq[q_idx,0] * time_d
                self.q[q_idx] += self.dq[q_idx] * time_d

            if time_elapsed.to_sec() > 0.05:
                m_id = 0
                m_id = self.publishRobotModelVis(m_id, col, links_fk)
                m_id = self.publishObstaclesVis(m_id, obst)
                last_time = rospy.Time.now()
                self.publishJointState()
                self.publishTransform(r_HAND_target, "hand_dest")

#            rospy.sleep(0.01)



if __name__ == '__main__':

    rospy.init_node('test_dynamic_model')

    task = TestDynamicModel()
    rospy.sleep(0.5)

    task.spin()


