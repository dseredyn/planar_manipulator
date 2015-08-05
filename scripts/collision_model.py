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
import xml.etree.ElementTree as ET

class CollisionModel:

    class Collision:
        def __init__(self):
            self.T_L_O = None
            self.type = None
            self.radius = None
            self.length = None

    class Link:
        def __init__(self):
            self.name = None
            self.col = None

    def readUrdfSrdf(self, urdf_filename, srdf_filename):
        def parseGeometryCollision2Element(col, geometry_elem):
            for child in geometry_elem:
                if child.tag == "sphere":
                    col.type = "sphere"
                    col.radius = float(child.attrib["radius"])
                elif child.tag == "capsule":
                    col.type = "capsule"
                    col.radius = float(child.attrib["radius"])
                    col.length = float(child.attrib["length"])
                else:
                    print "ERROR: parseGeometryCollision2Element: unknown element:", child.tag

        def parseCollision2Element(link, collision2_elem):
            col = CollisionModel.Collision()
            for child in collision2_elem:
                if child.tag == "origin":
                    rpy_str = child.attrib["rpy"].split()
                    xyz_str = child.attrib["xyz"].split()
                    col.T_L_O = PyKDL.Frame( PyKDL.Rotation.RPY(float(rpy_str[0]), float(rpy_str[1]), float(rpy_str[2])), PyKDL.Vector(float(xyz_str[0]), float(xyz_str[1]), float(xyz_str[2])) )
                elif child.tag == "geometry":
                    parseGeometryCollision2Element(col, child)
            return col

        def parseLinkElement(link_elem):
            link = CollisionModel.Link()
            link.name = link_elem.attrib["name"]
            for child in link_elem:
                if child.tag == "self_collision_checking":
                    col = parseCollision2Element(link, child)
                    if link.col == None:
                        link.col = []
                    link.col.append( col )
            return link

        # read the urdf file for convex collision primitives
        self.links = []
        self.link_map = {}
        tree = ET.parse(urdf_filename)
        root = tree.getroot()
        for child in root:
            if child.tag == "link":
                link = parseLinkElement(child)
                self.links.append( link )
                self.link_map[link.name] = link

        # read the srdf file
        tree = ET.parse(srdf_filename)
        root = tree.getroot()
        self.disabled_collision_pairs = []
        for child in root:
            if child.tag == "disable_collisions":
                self.disabled_collision_pairs.append( (child.attrib["link1"], child.attrib["link2"]) )

        self.collision_pairs = []
        for link1_idx in range(len(self.links)):
            if self.links[link1_idx].col == None or len(self.links[link1_idx].col) == 0:
                continue
            for link2_idx in range(link1_idx+1, len(self.links)):
                if self.links[link2_idx].col == None or len(self.links[link2_idx].col) == 0:
                    continue
                pair1 = (self.links[link1_idx].name, self.links[link2_idx].name)
                pair2 = (self.links[link2_idx].name, self.links[link1_idx].name)
                if not pair1 in self.disabled_collision_pairs and not pair2 in self.disabled_collision_pairs:
                    self.collision_pairs.append(pair1)

    def __init__(self):
        self.links = None
        self.link_map = None
        self.disabled_collision_pairs = None
        self.collision_pairs = None


