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

#include "planar_collision.h"

    void distancePoints(const KDL::Vector &pt1, const KDL::Vector &pt2, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        distance = (pt1-pt2).Norm();
        p_pt1 = pt1;
        p_pt2 = pt2;
    }

    void distanceLinePoint(const KDL::Vector &lineA, const KDL::Vector &lineB, const KDL::Vector &pt, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        KDL::Vector v = lineB - lineA;
        double ta = KDL::dot(v, lineA);
        double tb = KDL::dot(v, lineB);
        double tpt = KDL::dot(v, pt);
        if (tpt <= ta) {
            distance = (lineA-pt).Norm();
            p_pt1 = lineA;
            p_pt2 = pt;
        }
        else if (tpt >= tb) {
            distance = (lineB-pt).Norm();
            p_pt1 = lineB;
            p_pt2 = pt;
        }
        else {
            KDL::Vector n(v.y(), -v.x(), v.z());
            n.Normalize();
            double diff = KDL::dot(n, lineA) - KDL::dot(n, pt);
            distance = fabs(diff);
            p_pt1 = pt + (diff * n);
            p_pt2 = pt;
        }
    }

    void distancePointLine(const KDL::Vector &pt, const KDL::Vector &lineA, const KDL::Vector &lineB, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        distanceLinePoint(lineA, lineB, pt, distance, p_pt2, p_pt1);
    }

    void distanceLines(const KDL::Vector &l1A, const KDL::Vector &l1B, const KDL::Vector &l2A, const KDL::Vector &l2B, double &distance, KDL::Vector &p_pt1, KDL::Vector &p_pt2) {
        double x1 = l1A.x(), x2 = l1B.x(), x3 = l2A.x(), x4 = l2B.x();
        double y1 = l1A.y(), y2 = l1B.y(), y3 = l2A.y(), y4 = l2B.y();
        double denom = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
        // check if the lines cross
        if (denom != 0.0) {
            double xi = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4)) / denom;
            double yi = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4)) / denom;
            if (((xi >= x1 && xi <= x2) || (xi >= x2 && xi <= x1)) &&
                ((yi >= y1 && yi <= y2) || (yi >= y2 && yi <= y1)) &&
                ((xi >= x3 && xi <= x4) || (xi >= x4 && xi <= x3)) &&
                ((yi >= y3 && yi <= y4) || (yi >= y4 && yi <= y3))) {
                distance = 0.0;
                p_pt1 = KDL::Vector(xi, yi, 0.0);
                p_pt2 = KDL::Vector(xi, yi, 0.0);
                return;
            }
        }
        double tmp_distance[4];
        KDL::Vector tmp_p_pt1[4], tmp_p_pt2[4];
        distanceLinePoint(l1A, l1B, l2A, tmp_distance[0], tmp_p_pt1[0], tmp_p_pt2[0]);
        distanceLinePoint(l1A, l1B, l2B, tmp_distance[1], tmp_p_pt1[1], tmp_p_pt2[1]);
        distancePointLine(l1A, l2A, l2B, tmp_distance[2], tmp_p_pt1[2], tmp_p_pt2[2]);
        distancePointLine(l1B, l2A, l2B, tmp_distance[3], tmp_p_pt1[3], tmp_p_pt2[3]);
        
        double min_dist = 10000000.0;
        int min_idx = 0;
        for (int idx = 0; idx < 4; idx++) {
            if (tmp_distance[idx] < min_dist) {
                min_dist = tmp_distance[idx];
                min_idx = idx;
            }
        }
        distance = tmp_distance[min_idx];
        p_pt1 = tmp_p_pt1[min_idx];
        p_pt2 = tmp_p_pt2[min_idx];
    }

    bool checkCollision(const boost::shared_ptr<self_collision::Collision> &col1, const boost::shared_ptr<self_collision::Collision> &col2,
                        const KDL::Frame &T_B_L1, const KDL::Frame &T_B_L2, double activation_dist,
                        double &dist, KDL::Vector &p1_B, KDL::Vector &p2_B, KDL::Vector &n1_B, KDL::Vector &n2_B) {
                        KDL::Frame T_B_C1 = T_B_L1 * col1->origin;
                        KDL::Frame T_B_C2 = T_B_L2 * col2->origin;
                        boost::shared_ptr< self_collision::Geometry > geom1 = col1->geometry;
                        boost::shared_ptr< self_collision::Geometry > geom2 = col2->geometry;

                        double c_dist = (T_B_C1.p - T_B_C2.p).Norm();
                        double radius1, radius2;
                        if (geom1->type == self_collision::Geometry::CAPSULE && geom2->type == self_collision::Geometry::CAPSULE) {
                            self_collision::Capsule *capsule1 = static_cast<self_collision::Capsule* >(geom1.get());
                            self_collision::Capsule *capsule2 = static_cast<self_collision::Capsule* >(geom2.get());
                            radius1 = capsule1->radius;
                            radius2 = capsule2->radius;
                            if (c_dist - radius1 - radius2 - capsule1->length/2.0 - capsule2->length/2.0 > activation_dist) {
                                return false;
                            }
                            KDL::Vector line1A = T_B_C1 * KDL::Vector(0, 0, -capsule1->length/2);
                            KDL::Vector line1B = T_B_C1 * KDL::Vector(0, 0, capsule1->length/2);
                            KDL::Vector line2A = T_B_C2 * KDL::Vector(0, 0, -capsule2->length/2);
                            KDL::Vector line2B = T_B_C2 * KDL::Vector(0, 0, capsule2->length/2);
                            distanceLines(line1A, line1B, line2A, line2B, dist, p1_B, p2_B);
                        }
                        else if (geom1->type == self_collision::Geometry::CAPSULE && geom2->type == self_collision::Geometry::SPHERE) {
                            self_collision::Capsule *capsule1 = static_cast<self_collision::Capsule* >(geom1.get());
                            self_collision::Sphere *sphere2 = static_cast<self_collision::Sphere* >(geom2.get());
                            radius1 = capsule1->radius;
                            radius2 = sphere2->radius;
                            if (c_dist - radius1 - radius2 - capsule1->length/2.0 > activation_dist) {
                                return false;
                            }
                            KDL::Vector line1A = T_B_C1 * KDL::Vector(0, 0, -capsule1->length/2);
                            KDL::Vector line1B = T_B_C1 * KDL::Vector(0, 0, capsule1->length/2);
                            KDL::Vector pt2 = T_B_C2.p;
                            distanceLinePoint(line1A, line1B, pt2, dist, p1_B, p2_B);
                        }
                        else if (geom1->type == self_collision::Geometry::SPHERE && geom2->type == self_collision::Geometry::CAPSULE) {
                            self_collision::Sphere *sphere1 = static_cast<self_collision::Sphere* >(geom1.get());
                            self_collision::Capsule *capsule2 = static_cast<self_collision::Capsule* >(geom2.get());
                            radius1 = sphere1->radius;
                            radius2 = capsule2->radius;
                            if (c_dist - radius1 - radius2 - capsule2->length/2.0 > activation_dist) {
                                return false;
                            }
                            KDL::Vector pt1 = T_B_C1.p;
                            KDL::Vector line2A = T_B_C2 * KDL::Vector(0, 0, -capsule2->length/2);
                            KDL::Vector line2B = T_B_C2 * KDL::Vector(0, 0, capsule2->length/2);
                            distancePointLine(pt1, line2A, line2B, dist, p1_B, p2_B);
                        }
                        else if (geom1->type == self_collision::Geometry::SPHERE && geom2->type == self_collision::Geometry::SPHERE) {
                            self_collision::Sphere *sphere1 = static_cast<self_collision::Sphere* >(geom1.get());
                            self_collision::Sphere *sphere2 = static_cast<self_collision::Sphere* >(geom2.get());
                            radius1 = sphere1->radius;
                            radius2 = sphere2->radius;
                            if (c_dist - radius1 - radius2 > activation_dist) {
                                return false;
                            }
                            KDL::Vector pt1 = T_B_C1.p;
                            KDL::Vector pt2 = T_B_C2.p;
                            distancePoints(pt1, pt2, dist, p1_B, p2_B);
                        }
                        else {
                            std::cout << "ERROR: checkCollision: unknown object type" << std::endl;
                            return false;
                        }

                        if (dist - radius1 - radius2 <= activation_dist) {
                            dist = dist - radius1 - radius2;
                            KDL::Vector v = p2_B - p1_B;
                            v.Normalize();
                            n1_B = v;
                            n2_B = -v;
                            p1_B = p1_B + n1_B * radius1;
                            p2_B = p2_B + n2_B * radius2;

                            return true;
                        }

                        return false;
    }

    void getCollisionPairs(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                            double activation_dist, std::vector<CollisionInfo> &link_collisions) {
        // self collision
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it = col_model->enabled_collisions.begin(); it != col_model->enabled_collisions.end(); it++) {
            int link1_idx = it->first;
            int link2_idx = it->second;
            KDL::Frame T_B_L1 = links_fk[link1_idx];
            KDL::Frame T_B_L2 = links_fk[link2_idx];

            for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
                for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
                    double dist = 0.0;
                    KDL::Vector p1_B, p2_B, n1_B, n2_B;
                    if (checkCollision(*col1, *col2, T_B_L1, T_B_L2, activation_dist, dist, p1_B, p2_B, n1_B, n2_B)) {
                        CollisionInfo col_info;
                        col_info.link1_idx = link1_idx;
                        col_info.link2_idx = link2_idx;
                        col_info.dist = dist;
                        col_info.n1_B = n1_B;
                        col_info.n2_B = n2_B;
                        col_info.p1_B = p1_B;
                        col_info.p2_B = p2_B;
                        link_collisions.push_back(col_info);
                    }
                }
            }
        }
    }

    bool checkSelfCollision(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk, double activation_dist) {
        int env_link_idx = col_model->getLinkIndex("env_link");

        // self collision
        for (self_collision::CollisionModel::CollisionPairs::const_iterator it = col_model->enabled_collisions.begin(); it != col_model->enabled_collisions.end(); it++) {
            int link1_idx = it->first;
            int link2_idx = it->second;
            if (link1_idx == env_link_idx || link2_idx == env_link_idx) {
                continue;
            }
            KDL::Frame T_B_L1 = links_fk[link1_idx];
            KDL::Frame T_B_L2 = links_fk[link2_idx];

            for (self_collision::Link::VecPtrCollision::const_iterator col1 = col_model->getLinkCollisionArray(link1_idx).begin(); col1 != col_model->getLinkCollisionArray(link1_idx).end(); col1++) {
                for (self_collision::Link::VecPtrCollision::const_iterator col2 = col_model->getLinkCollisionArray(link2_idx).begin(); col2 != col_model->getLinkCollisionArray(link2_idx).end(); col2++) {
                    double dist = 0.0;
                    KDL::Vector p1_B, p2_B, n1_B, n2_B;
                    if (checkCollision(*col1, *col2, T_B_L1, T_B_L2, activation_dist, dist, p1_B, p2_B, n1_B, n2_B)) {
                        return true;
//                        CollisionInfo col_info;
//                        col_info.link1_idx = link1_idx;
//                        col_info.link2_idx = link2_idx;
//                        col_info.dist = dist;
//                        col_info.n1_B = n1_B;
//                        col_info.n2_B = n2_B;
//                        col_info.p1_B = p1_B;
//                        col_info.p2_B = p2_B;
//                        link_collisions.push_back(col_info);
                    }
                }
            }
        }
        return false;
    }

    void getRepulsiveForces(const boost::shared_ptr<self_collision::CollisionModel> &col_model, const std::vector<KDL::Frame > &links_fk,
                            double activation_dist, const KDL::Vector &center, std::vector<CollisionInfo> &link_collisions) {

        boost::shared_ptr< self_collision::Collision > pcol(new self_collision::Collision());
        pcol->geometry.reset(new self_collision::Sphere());
        boost::shared_ptr<self_collision::Sphere > sph = boost::static_pointer_cast<self_collision::Sphere >(pcol->geometry);
        sph->radius = 0.02;
        pcol->origin = KDL::Frame(center);

        int env_link_idx = col_model->getLinkIndex("env_link");
        int base_link_idx = col_model->getLinkIndex("base");

        const KDL::Frame &T_B_L1 = links_fk[base_link_idx];

        for (self_collision::CollisionModel::VecPtrLink::const_iterator it = col_model->getLinks().begin(); it != col_model->getLinks().end(); it++) {
            int link_idx = (*it)->index_;
            if (link_idx == env_link_idx || link_idx == base_link_idx) {
                continue;
            }
            const KDL::Frame &T_B_L2 = links_fk[link_idx];

            for (self_collision::Link::VecPtrCollision::const_iterator col = col_model->getLinkCollisionArray(link_idx).begin(); col != col_model->getLinkCollisionArray(link_idx).end(); col++) {
                double dist = 0.0;
                KDL::Vector p1_B, p2_B, n1_B, n2_B;
                if (checkCollision(pcol, *col, T_B_L1, T_B_L2, activation_dist, dist, p1_B, p2_B, n1_B, n2_B)) {
                    CollisionInfo col_info;
                    col_info.link1_idx = base_link_idx;
                    col_info.link2_idx = link_idx;
                    col_info.dist = dist;
                    col_info.n1_B = n1_B;
                    col_info.n2_B = n2_B;
                    col_info.p1_B = p1_B;
                    col_info.p2_B = p2_B;
                    link_collisions.push_back(col_info);
                }
            }
        }
    }

