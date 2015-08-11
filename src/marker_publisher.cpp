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

#include "marker_publisher.h"
#include <iostream>
#include "geometry_msgs/Point.h"


MarkerPublisher::MarkerPublisher(ros::NodeHandle &nh) :
    nh_(nh)
{
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/velma_markers", 1000);
}

MarkerPublisher::~MarkerPublisher() {
}

void MarkerPublisher::publish() {
    pub_.publish( marker_array_ );
    marker_array_.markers.clear();
}

int MarkerPublisher::addSinglePointMarker(int m_id, const KDL::Vector &pos, double r, double g, double b, double a, double size, const std::string &frame_id) {
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
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker_array_.markers.push_back(marker);
    return m_id + 1;
}

int MarkerPublisher::addVectorMarker(int m_id, const KDL::Vector &v1, const KDL::Vector &v2, double r, double g, double b, double a, double size, const std::string &frame_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.ns = "default";
    marker.id = m_id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::Point p1, p2;
    p1.x = v1.x();
    p1.y = v1.y();
    p1.z = v1.z();
    p2.x = v2.x();
    p2.y = v2.y();
    p2.z = v2.z();
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.scale.x = size;
    marker.scale.y = 2.0 * size;
    marker.scale.z = 0.0;
    marker.color.a = a;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker_array_.markers.push_back(marker);
    return m_id + 1;
}

int MarkerPublisher::addCapsule(int m_id, const KDL::Frame &fr, double length, double radius, const std::string &frame_id) {

	KDL::Vector zero;
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
	marker_array_.markers.push_back(marker);

	visualization_msgs::Marker marker2(marker);
	marker2.id = m_id+1;
	marker2.pose.position.x = fr.p.x() + v2.x()/2.0;
	marker2.pose.position.y = fr.p.y() + v2.y()/2.0;
	marker2.pose.position.z = fr.p.z() + v2.z()/2.0;
	if (length > 0.0001)
		marker_array_.markers.push_back(marker2);

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
		marker_array_.markers.push_back(marker3);

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
	marker_array_.markers.push_back(marker4);
	return m_id + 4;
}

void MarkerPublisher::addEraseMarkers(int from, int to)
{
	for (int i=from; i<to; i++)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "world";
		marker.header.stamp = ros::Time();
		marker.ns = "default";
		marker.id = i;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::DELETE;
		marker_array_.markers.push_back(marker);
	}
}

