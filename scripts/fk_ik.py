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

import PyKDL
import numpy as np
import math

import urdf_parser_py
from urdf_parser_py.urdf import URDF
import pykdl_utils.kdl_parser as kdl_urdf

class FkIkSolver:

    def kdl_tree_from_urdf_model(self, urdf, js_inactive_names_vector, js_pos):
        segment_map = {}
        segment_id = 0
        segment_name_id_map = {}
        segment_parent_map = {}
        root = urdf.get_root()
        tree = PyKDL.Tree(root)

        segment_map[segment_id] = None
        segment_parent_map[segment_id] = None
        segment_name_id_map[root] = segment_id
        segment_id += 1

        def add_children_to_tree(parent, segment_id):
            if parent in urdf.child_map:
                for joint, child_name in urdf.child_map[parent]:
                    if joint in js_inactive_names_vector:
                        print "setting as fixed:", joint, js_pos[joint]
                        joint_rot = -js_pos[joint]
                        urdf.joint_map[joint].joint_type = 'fixed'
                    else:
                        joint_rot = 0.0
                    child = urdf.link_map[child_name]
                    if child.inertial is not None:
                        kdl_inert = kdl_urdf.urdf_inertial_to_kdl_rbi(child.inertial)
                    else:
                        kdl_inert = PyKDL.RigidBodyInertia()
                    kdl_jnt = kdl_urdf.urdf_joint_to_kdl_joint(urdf.joint_map[joint])
                    kdl_origin = kdl_urdf.urdf_pose_to_kdl_frame(urdf.joint_map[joint].origin) * PyKDL.Frame(PyKDL.Rotation.RotZ(joint_rot))
                    kdl_sgm = PyKDL.Segment(child_name, kdl_jnt,
                                          kdl_origin, kdl_inert)

                    segment_map[segment_id] = kdl_sgm
                    segment_parent_map[segment_id] = segment_name_id_map[parent]
                    segment_name_id_map[child_name] = segment_id
                    segment_id += 1

                    tree.addSegment(kdl_sgm, parent)
                    segment_id = add_children_to_tree(child_name, segment_id)
            return segment_id
        add_children_to_tree(root, segment_id)
        return tree, segment_map, segment_parent_map, segment_name_id_map

    def createSegmentToJointMap(self, joint_names_vector, inactive_joint_names):
        self.joint_names_vector = joint_names_vector
        self.segment_id_q_id_map = {}
        for q_idx in range(len(joint_names_vector)):
            joint_name = joint_names_vector[q_idx]
            for seg_id in self.segment_map:
                seg = self.segment_map[seg_id]
                if seg == None:
                    continue
                if joint_name == seg.getJoint().getName():
                    self.segment_id_q_id_map[seg_id] = q_idx

        self.inactive_segment_id_q_id_map = {}
        for q_idx in range(len(inactive_joint_names)):
            joint_name = inactive_joint_names[q_idx]
            for seg_id in self.segment_map:
                seg = self.segment_map[seg_id]
                if seg == None:
                    continue
                if joint_name == seg.getJoint().getName():
                    self.inactive_segment_id_q_id_map[seg_id] = q_idx

    class FkSolver:
        def __init__(self, tree, base_name, end_name, joint_names_vector):
            self.chain = tree.getChain(base_name, end_name)
            self.jac_solver = PyKDL.ChainJntToJacSolver( self.chain )
            self.q_indices_map = []
            self.fk_solver = PyKDL.ChainFkSolverPos_recursive(self.chain)
            for chain_q_idx in range(self.chain.getNrOfSegments()):
                joint = self.chain.getSegment(chain_q_idx).getJoint()
                chain_joint_name = joint.getName()
                chain_joint_type = joint.getType()
                if chain_joint_type == PyKDL.Joint.None:
                    continue
                q_idx = 0
                for joint_name in joint_names_vector:
                    if joint_name == chain_joint_name:
                        self.q_indices_map.append(q_idx)
                        break
                    q_idx += 1
                if q_idx == len(joint_names_vector):
                    print "ERROR: createJacobianSolver", chain_joint_name, " not in", joint_names_vector
                    exit(0)
            self.chain_length = len(self.q_indices_map)

    def calculateFk(self, base_name, end_name, q):
        if not (base_name, end_name) in self.fk_solvers:
            self.fk_solvers[(base_name, end_name)] = FkIkSolver.FkSolver(self.tree, base_name, end_name, self.joint_names_vector)

        fk_solver = self.fk_solvers[(base_name, end_name)]

        q_fk = PyKDL.JntArray( fk_solver.chain_length )
        q_fk_idx = 0
        for q_idx in fk_solver.q_indices_map:
            q_fk[q_fk_idx] = q[q_idx]
            q_fk_idx += 1
        fr = PyKDL.Frame()
        fk_solver.fk_solver.JntToCart(q_fk, fr)
        return fr

    def getJacobian(self, base_name, end_name, q):
        if not (base_name, end_name) in self.fk_solvers:
            self.fk_solvers[(base_name, end_name)] = FkIkSolver.FkSolver(self.tree, base_name, end_name, self.joint_names_vector)

        fk_solver = self.fk_solvers[(base_name, end_name)]

        # extract joint values for the chain
        q_jac = PyKDL.JntArray( fk_solver.chain_length )
        q_jac_idx = 0
        for q_idx in fk_solver.q_indices_map:
            q_jac[q_jac_idx] = q[q_idx]
            q_jac_idx += 1
        jac_small = PyKDL.Jacobian( fk_solver.chain.getNrOfJoints() )
        fk_solver.jac_solver.JntToJac(q_jac, jac_small)

        # create the jacobian for all joints
        jac_big = np.matrix(np.zeros( (6, len(q)) ))

        for col_idx in range(jac_small.columns()):
            q_idx = fk_solver.q_indices_map[col_idx]
            col = jac_small.getColumn(col_idx)
            for row_idx in range(6):
                jac_big[row_idx, q_idx] = col[row_idx]
        return jac_big

    def getJacobianForX(self, jac, link_name, x, q, iq, base_name='torso_base'):
        link_index = self.segment_name_id_map[link_name]
        # Lets search the tree-element
        # If segmentname is not inside the tree, back out:
        # Let's make the jacobian zero:
        for q_idx in range(len(q)):
            jac.setColumn(q_idx, PyKDL.Twist())

        T_total = PyKDL.Frame(x)
        root_index = self.segment_name_id_map[base_name]
        l_index = link_index
        # Lets recursively iterate until we are in the root segment
        while l_index != root_index:
            # get the corresponding q_nr for this TreeElement:
            # get the pose of the segment:
            seg_kdl = self.segment_map[l_index]
            if seg_kdl.getJoint().getType() == PyKDL.Joint.None:
                q_idx = None
                q_seg = 0.0
            elif l_index in self.segment_id_q_id_map:
                try:
                    q_idx = self.segment_id_q_id_map[l_index]
                except KeyError as ke:
                    print ke.errno, ke.strerror
                    print "joint type", seg_kdl.getJoint().getType(), " joint name", seg_kdl.getJoint().getName()
                    exit(0)
                q_seg = q[q_idx]
            else:
                q_idx = self.inactive_segment_id_q_id_map[l_index]
                q_seg = iq[q_idx]

            T_local = seg_kdl.pose(q_seg)
            # calculate new T_end:
            T_total = T_local * T_total
            # get the twist of the segment:
            t_local = self.segment_map[l_index].twist(q_seg, 1.0)
            # transform the endpoint of the local twist to the global endpoint:
            t_local = t_local.RefPoint(T_total.p - T_local.p)
            # transform the base of the twist to the endpoint
            t_local = T_total.M.Inverse(t_local)
            # store the twist in the jacobian:
            if q_idx != None:
                jac.setColumn(q_idx,t_local)
            else:
                if t_local.vel.Norm() > 0.000001 or t_local.rot.Norm() > 0.000001:
                    print "ERROR: JntToJac t_local != 0", t_local
                    exit(0)
            # goto the parent
            l_index = self.segment_parent_map[l_index]
        # Change the base of the complete jacobian from the endpoint to the base
#        changeBase(jac, T_total.M, jac);
#        jac.changeBase(T_total.M)
        return 0;

    def getJacobiansForPairX(self, jac1, jac2, link_name1, x1, link_name2, x2, q, iq):
        # get the first common link
        link_index1 = self.segment_name_id_map[link_name1]
        l_index = link_index1
        link1_chain = set()
        while True:
            link1_chain.add(l_index)
            if l_index in self.segment_parent_map:
                l_index = self.segment_parent_map[l_index]
            else:
                break

        link_index2 = self.segment_name_id_map[link_name2]
        l_index = link_index2
        while True:
            if l_index in link1_chain:
                break
            if l_index in self.segment_parent_map:
                l_index = self.segment_parent_map[l_index]
            else:
                # this is unexpected
                return None

        common_link_name = self.segment_id_name_map[l_index]
        self.getJacobianForX(jac1, link_name1, x1, q, iq, base_name=common_link_name)
        self.getJacobianForX(jac2, link_name2, x2, q, iq, base_name=common_link_name)
        return common_link_name

    def __init__(self, joint_names_vector, inactive_joint_names, js_pos):
        self.robot = URDF.from_parameter_server()
        self.tree, self.segment_map, self.segment_parent_map, self.segment_name_id_map = self.kdl_tree_from_urdf_model(self.robot, inactive_joint_names, js_pos)
        self.segment_id_name_map = {}
        for seg_name in self.segment_name_id_map:
            seg_id = self.segment_name_id_map[seg_name]
            self.segment_id_name_map[seg_id] = seg_name

        self.fk_solvers = {}

        self.createSegmentToJointMap(joint_names_vector, inactive_joint_names)

        joint_limit_map = {}
        for j in self.robot.joints:
            if j.limit != None:
                joint_limit_map[j.name] = j.limit

        self.lim_lower = np.empty(len(joint_names_vector))
        self.lim_lower_soft = np.empty(len(joint_names_vector))
        self.lim_upper = np.empty(len(joint_names_vector))
        self.lim_upper_soft = np.empty(len(joint_names_vector))
        q_idx = 0
        for joint_name in joint_names_vector:
            self.lim_lower[q_idx] = joint_limit_map[joint_name].lower
            self.lim_lower_soft[q_idx] = self.lim_lower[q_idx] + 15.0/180.0*math.pi
            self.lim_upper[q_idx] = joint_limit_map[joint_name].upper
            self.lim_upper_soft[q_idx] = self.lim_upper[q_idx] - 15.0/180.0*math.pi
            q_idx += 1

