//
//  triangle3.h
//  World Maker
//
//  Created by Jerome Johnson on 4/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef triangle3_h
#define triangle3_h

#include <algorithm>
#include "vector3.h"
#include "spline.h"
#include "plane.h"
#include "triangle.h"

namespace motu{
    struct Triangle3{
        Vector3 vertices[3];
        
        Triangle3(){}
        
        Triangle3(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
            vertices[0] = a;
            vertices[1] = b;
            vertices[2] = c;
        }
        
		Triangle3(const Triangle3 &tri) {
            std::copy(tri.vertices, tri.vertices + 3, vertices);
        }

		Triangle toTriangle2() const{
			Triangle out;
			for (size_t i = 0; i != 3; ++i) {
				out.vertices[i] = vertices[i].toVector2();
			}
			return out;
		}
        
        Vector3 normal() const{
            return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0])/*.normalized()*/;
        }
        
        void getSplines(Spline *splines) const{
            splines[0] = Spline(vertices[0], vertices[1]);
            splines[1] = Spline(vertices[1], vertices[2]);
            splines[2] = Spline(vertices[2], vertices[0]);
        }

		void flipRotation() {
			std::swap(vertices[0], vertices[2]);
		}
        
        template <class Itr>
        static Vector3 computeNormal(const Vector3 &vertex, Itr begin, Itr end){
            Itr i = begin;
            Itr last = begin;
            Vector3 total(0.0f, 0.0f, 0.0f);
            for (++i; i != end; ++i){
                Vector3 normal(Triangle3(vertex, *i, *last).normal());
                if (normal.z < 0.0f){
                    normal *= -1.0f;
                }
                total += normal;
                last = i;
            }
            return total.normalized();
        }

		bool intersection(const Spline &spline, Vector3 &intersection) {
			Plane plane(vertices[0], normal());
			if (plane.intersection(spline.endA, spline.direction(), intersection)) {
				/*Vector3 an(Triangle3(vertices[0], vertices[1], intersection).normal().normalized());
				std::cout << an << " - "  << normal().normalized() << std::endl;*/
				float a = (vertices[1] - vertices[0]).cross(intersection - vertices[0]).dot(plane.normal),
					b = (vertices[2] - vertices[1]).cross(intersection - vertices[1]).dot(plane.normal);
				if ((a < 0.0f && b > 0.0f) || (a > 0.0f && b < 0.0f)){
					return false;
				}
				b = (vertices[0] - vertices[2]).cross(intersection - vertices[2]).dot(plane.normal);
				return (a > 0.0f && b > 0.0f) || (a < 0.0f && b < 0.0f);
			}
		}

		bool intersects(const Plane &plane) const{
			for (int i = 0; i != 3; ++i) {
				if (Spline(vertices[i], vertices[(i + 1) % 3]).intersects(plane)) {
					return true;
				}
			}
			return false;
		}

		static void emplaceWithRotation(const Vector3 &normal, const Vector3 &a, const Vector3 &b, const Vector3 &c, std::vector<Triangle3> &out) {
			if ((b - a).cross(c - b).dot(normal) < 0.0f) {
				out.emplace_back(c, b, a);
			}
			else {
				out.emplace_back(a, b, c);
			}
		}

		std::vector<Triangle3> &slice(const Plane &plane, std::vector<Triangle3> &out) const {
			Spline splines[3];
			getSplines(splines);
			std::pair<Vector3, const Spline*> intersection[3];
			Vector3 normal(this->normal());
			size_t offset = 0;
			const Vector3 *a, *b, *c, *d, *e;
			for (int i = 0; i != 3; ++i) {
				if (splines[i].intersects(plane, intersection[offset].first)) {
					intersection[offset++].second = splines + i;
				}
			}
			switch (offset) {
			case 2:
				if (intersection[0].second->endA == intersection[1].second->endA) {
					a = &intersection[0].second->endA;
					b = &intersection[0].first;
					c = &intersection[0].second->endB;
					d = &intersection[1].second->endB;
					e = &intersection[1].first;
				}
				else if (intersection[0].second->endA == intersection[1].second->endB) {
					a = &intersection[0].second->endA;
					b = &intersection[0].first;
					c = &intersection[0].second->endB;
					d = &intersection[1].second->endA;
					e = &intersection[1].first;
				}
				else if (intersection[0].second->endB == intersection[1].second->endA) {
					a = &intersection[1].second->endA;
					b = &intersection[1].first;
					c = &intersection[1].second->endB;
					d = &intersection[0].second->endA;
					e = &intersection[0].first;
				}
				else if (intersection[0].second->endB == intersection[1].second->endB) {
					a = &intersection[0].second->endB;
					b = &intersection[0].first;
					c = &intersection[0].second->endA;
					d = &intersection[1].second->endA;
					e = &intersection[1].first;
				}
				emplaceWithRotation(normal, *a, *b, *e, out);
				emplaceWithRotation(normal, *b, *c, *d, out);
				emplaceWithRotation(normal, *e, *b, *d, out);
				break;
			case 3:
				if (intersection[0].first == intersection[1].first) {
					emplaceWithRotation(normal, intersection[0].second->endA, intersection[2].first, intersection[0].second->endB, out);;
					emplaceWithRotation(normal, intersection[1].second->endA, intersection[2].first, intersection[1].second->endB, out);
				}
				else if (intersection[0].first == intersection[2].first) {
					emplaceWithRotation(normal, intersection[0].second->endA, intersection[1].first, intersection[0].second->endB, out);
					emplaceWithRotation(normal, intersection[2].second->endA, intersection[1].first, intersection[2].second->endB, out);
				}
				else {
					emplaceWithRotation(normal, intersection[1].second->endA, intersection[0].first, intersection[1].second->endB, out);
					emplaceWithRotation(normal, intersection[2].second->endA, intersection[0].first, intersection[2].second->endB, out);
				}
			}
			return out;
		}

		Vector3 baricentre() const{
			Vector3 total(0.0f, 0.0f, 0.0f);
			for (size_t i = 0; i != 3; total += vertices[i++]);
			return total / 3.0f;
		}
    };
    
    struct Triangle3WithNormals : public Triangle3{
        Vector3 normals[3];
        
        Triangle3WithNormals(){}
        
        Triangle3WithNormals(const Vector3 &a, const Vector3 &b, const Vector3 &c) : Triangle3(a, b, c){}
        
        Triangle3WithNormals(const Triangle3 &verts) : Triangle3(verts){}
    };
}

#endif /* triangle3_h */
