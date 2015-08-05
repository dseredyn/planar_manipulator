/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Dawid Seredynski */

#include "marker_publisher.h"
#include <iostream>
#include "geometry_msgs/Point.h"

int publishSinglePointMarker(ros::Publisher &pub, int m_id, const KDL::Vector &pos, double r, double g, double b, double size, const std::string &frame_id)
{
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos.x();
	marker.pose.position.y = pos.y();
	marker.pose.position.z = pos.z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker_array.markers.push_back(marker);
	pub.publish( marker_array );
	return m_id + 1;
}

int publishLineMarker(ros::Publisher &pub, int m_id, const KDL::Vector &pos1, const KDL::Vector &pos2, double r, double g, double b)
{
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.points.resize(2);
	marker.points[0].x = pos1.x();
	marker.points[0].y = pos1.y();
	marker.points[0].z = pos1.z();
	marker.points[1].x = pos2.x();
	marker.points[1].y = pos2.y();
	marker.points[1].z = pos2.z();
	marker.scale.x = 0.01;
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker_array.markers.push_back(marker);

	visualization_msgs::Marker marker2;
	marker2.header.frame_id = "world";
	marker2.header.stamp = ros::Time();
	marker2.ns = "default";
	marker2.id = m_id+1;
	marker2.type = visualization_msgs::Marker::SPHERE;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.position.x = pos1.x();
	marker2.pose.position.y = pos1.y();
	marker2.pose.position.z = pos1.z();
	marker2.pose.orientation.x = 0.0;
	marker2.pose.orientation.y = 0.0;
	marker2.pose.orientation.z = 0.0;
	marker2.pose.orientation.w = 1.0;
	marker2.scale.x = 0.02;
	marker2.scale.y = 0.02;
	marker2.scale.z = 0.02;
	marker2.color.a = 1.0;
	marker2.color.r = 1;
	marker2.color.g = 0;
	marker2.color.b = 0;
	marker_array.markers.push_back(marker2);

	visualization_msgs::Marker marker3;
	marker3.header.frame_id = "world";
	marker3.header.stamp = ros::Time();
	marker3.ns = "default";
	marker3.id = m_id+2;
	marker3.type = visualization_msgs::Marker::SPHERE;
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.pose.position.x = pos2.x();
	marker3.pose.position.y = pos2.y();
	marker3.pose.position.z = pos2.z();
	marker3.pose.orientation.x = 0.0;
	marker3.pose.orientation.y = 0.0;
	marker3.pose.orientation.z = 0.0;
	marker3.pose.orientation.w = 1.0;
	marker3.scale.x = 0.02;
	marker3.scale.y = 0.02;
	marker3.scale.z = 0.02;
	marker3.color.a = 1.0;
	marker3.color.r = 0;
	marker3.color.g = 1;
	marker3.color.b = 0;
	marker_array.markers.push_back(marker3);

	pub.publish( marker_array );
	return m_id + 3;
}

int publishCapsule(ros::Publisher &pub, int m_id, const KDL::Frame &fr, double length, double radius, const std::string &frame_id)
{
	visualization_msgs::MarkerArray marker_array;

	KDL::Vector zero;
//	KDL::Vector v(0,0,length);
	KDL::Vector v(0,length,0);
	KDL::Vector v2 = (fr * v) - (fr * zero);

	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = fr.p.x() - v2.x()/2.0;
	marker.pose.position.y = fr.p.y() - v2.y()/2.0;
	marker.pose.position.z = fr.p.z() - v2.z()/2.0;
	double qx, qy, qz, qw;
	fr.M.GetQuaternion(qx, qy, qz, qw);
	marker.pose.orientation.x = qx;
	marker.pose.orientation.y = qy;
	marker.pose.orientation.z = qz;
	marker.pose.orientation.w = qw;
	marker.scale.x = radius * 2.0;
	marker.scale.y = radius * 2.0;
	marker.scale.z = radius * 2.0;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker_array.markers.push_back(marker);

	visualization_msgs::Marker marker2(marker);
	marker2.id = m_id+1;
	marker2.pose.position.x = fr.p.x() + v2.x()/2.0;
	marker2.pose.position.y = fr.p.y() + v2.y()/2.0;
	marker2.pose.position.z = fr.p.z() + v2.z()/2.0;
	if (length > 0.0001)
		marker_array.markers.push_back(marker2);

	visualization_msgs::Marker marker3(marker);
	marker3.id = m_id+2;
	marker3.type = visualization_msgs::Marker::CYLINDER;
	marker3.pose.position.x = fr.p.x();
	marker3.pose.position.y = fr.p.y();
	marker3.pose.position.z = fr.p.z();
	marker3.scale.z = length;
    KDL::Frame T_O_C(KDL::Rotation::RotX(90.0/180.0*3.1415));
    KDL::Frame T_B_C = fr * T_O_C;
	T_B_C.M.GetQuaternion(qx, qy, qz, qw);
	marker3.pose.orientation.x = qx;
	marker3.pose.orientation.y = qy;
	marker3.pose.orientation.z = qz;
	marker3.pose.orientation.w = qw;
	if (length > 0.0001)
		marker_array.markers.push_back(marker3);

	visualization_msgs::Marker marker4(marker);
	marker4.id = m_id+3;
	marker4.type = visualization_msgs::Marker::SPHERE;
	marker4.pose.position.x = fr.p.x();
	marker4.pose.position.y = fr.p.y();
	marker4.pose.position.z = fr.p.z();
	marker4.scale.x = 0.01;
	marker4.scale.y = 0.01;
	marker4.scale.z = 0.01;
	marker4.color.a = 1.0;
	marker4.color.r = 0.0;
	marker4.color.g = 1.0;
	marker4.color.b = 0.0;
	marker_array.markers.push_back(marker4);

	pub.publish( marker_array );
	return m_id + 4;
}

int publishCylinder(ros::Publisher &pub, int m_id, KDL::Frame fr, double length, double radius)
{
	visualization_msgs::MarkerArray marker_array;

	KDL::Vector zero;
	KDL::Vector v(0,0,length);
	KDL::Vector v2 = (fr * v) - (fr * zero);

	visualization_msgs::Marker marker;
	marker.header.frame_id = "world";//"torso_base";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = fr.p.x() - v2.x()/2.0;
	marker.pose.position.y = fr.p.y() - v2.y()/2.0;
	marker.pose.position.z = fr.p.z() - v2.z()/2.0;
	double qx, qy, qz, qw;
	fr.M.GetQuaternion(qx, qy, qz, qw);
	marker.pose.orientation.x = qx;
	marker.pose.orientation.y = qy;
	marker.pose.orientation.z = qz;
	marker.pose.orientation.w = qw;
	marker.scale.x = radius * 2.0;
	marker.scale.y = radius * 2.0;
	marker.scale.z = length;
	marker.color.a = 0.5;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	marker_array.markers.push_back(marker);

	pub.publish( marker_array );
	return m_id + 1;
}
/*
int publishMeshMarker(ros::Publisher &pub, int m_id, const KDL::Frame &tf, const fcl_2::Vec3f *points, int num_planes, const int *polygons, double r, double g, double b)
{
	visualization_msgs::MarkerArray marker_array;

	visualization_msgs::Marker marker;
	visualization_msgs::Marker marker2;

	// triangles
	marker.header.frame_id = "world";
	marker.header.stamp = ros::Time();
	marker.ns = "default";
	marker.id = m_id;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = tf.p.x();
	marker.pose.position.y = tf.p.y();
	marker.pose.position.z = tf.p.z();
	double qx, qy, qz, qw;
	tf.M.GetQuaternion(qx, qy, qz, qw);
	marker.pose.orientation.x = qx;
	marker.pose.orientation.y = qy;
	marker.pose.orientation.z = qz;
	marker.pose.orientation.w = qw;
	int poly_idx = 0;
	for (int i=0; i<num_planes; i++)
	{
		for (int j=0; j<3; j++)
		{
			geometry_msgs::Point pt;
			pt.x = points[polygons[poly_idx + 1 + j]][0];
			pt.y = points[polygons[poly_idx + 1 + j]][1];
			pt.z = points[polygons[poly_idx + 1 + j]][2];
			marker.points.push_back(pt);
//			ROS_INFO("num_planes: %d  poly_idx: %d   i: %d   j: %d   polygons[i*4 + 1 + j]: %d", num_planes, poly_idx, i, j, polygons[poly_idx + 1 + j]);
		}
		poly_idx += polygons[poly_idx]+1;
	}
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 0.5;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker_array.markers.push_back(marker);

	// edges
	marker2.header.frame_id = "world";
	marker2.header.stamp = ros::Time();
	marker2.ns = "default";
	marker2.id = m_id+1;
	marker2.type = visualization_msgs::Marker::LINE_LIST;
	marker2.action = visualization_msgs::Marker::ADD;
	marker2.pose.position.x = tf.p.x();
	marker2.pose.position.y = tf.p.y();
	marker2.pose.position.z = tf.p.z();
	tf.M.GetQuaternion(qx, qy, qz, qw);
	marker2.pose.orientation.x = qx;
	marker2.pose.orientation.y = qy;
	marker2.pose.orientation.z = qz;
	marker2.pose.orientation.w = qw;
	poly_idx = 0;
	for (int i=0; i<num_planes; i++)
	{
		int points_in_poly = polygons[poly_idx];
		for (int j=0; j<points_in_poly; j++)
		{
			geometry_msgs::Point pt1;
			pt1.x = points[polygons[poly_idx + 1 + j]][0];
			pt1.y = points[polygons[poly_idx + 1 + j]][1];
			pt1.z = points[polygons[poly_idx + 1 + j]][2];
			marker2.points.push_back(pt1);

			geometry_msgs::Point pt2;
			pt2.x = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][0];
			pt2.y = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][1];
			pt2.z = points[polygons[poly_idx + 1 + (j+1)%points_in_poly]][2];
			marker2.points.push_back(pt2);
		}
		poly_idx += points_in_poly+1;
	}
	marker2.scale.x = 0.005;
	marker2.color.a = 1;
	marker2.color.r = 1-r;
	marker2.color.g = 1-g;
	marker2.color.b = 1-b;

	marker_array.markers.push_back(marker2);
	pub.publish( marker_array );
	return m_id + 2;
}
*/
void clearMarkers(ros::Publisher &pub, int from, int to)
{
	visualization_msgs::MarkerArray marker_array;
	for (int i=from; i<to; i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time();
		marker.ns = "default";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::DELETE;
		marker_array.markers.push_back(marker);
	}
	pub.publish( marker_array );
}


