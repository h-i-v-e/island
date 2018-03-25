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
				out.vertices[i] = vertices[i].asVector2();
			}
			return out;
		}
        
        Vector3 normal() const{
            return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
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

		bool intersection(const Spline &spline, Vector3 &intersection) const;

		bool intersects(const Plane &plane) const{
			for (int i = 0; i != 3; ++i) {
				if (Spline(vertices[i], vertices[(i + 1) % 3]).intersects(plane)) {
					return true;
				}
			}
			return false;
		}

		std::vector<Triangle3> &slice(const Plane &plane, std::vector<Triangle3> &out) const;

		Vector3 baricentre() const {
			Vector3 total(0.0f, 0.0f, 0.0f);
			for (size_t i = 0; i != 3; total += vertices[i++]);
			return total / 3.0f;
		}
    };
    
    struct Triangle3WithNormals : public Triangle3{
        Vector3 normals[3];
        
        Triangle3WithNormals(){}
        
        Triangle3WithNormals(const Vector3 &a, const Vector3 &b, const Vector3 &c) : Triangle3(a, b, c){}

		Triangle3WithNormals(const Vector3 &a, const Vector3 &b, const Vector3 &c, const Vector3 &na, const Vector3 &nb, const Vector3 &nc) : Triangle3(a, b, c) {
			normals[0] = na;
			normals[1] = nb;
			normals[2] = nc;
		}
        
        Triangle3WithNormals(const Triangle3 &verts) : Triangle3(verts){}

		std::vector<Triangle3WithNormals> &slice(const Plane &plane, std::vector<Triangle3WithNormals> &out) const;

		SplineWithNormals *getSplines(SplineWithNormals *splines) const {
			splines[0] = SplineWithNormals(vertices[0], vertices[1], normals[0], normals[1]);
			splines[1] = SplineWithNormals(vertices[1], vertices[2], normals[1], normals[2]);
			splines[2] = SplineWithNormals(vertices[2], vertices[0], normals[2], normals[0]);
			return splines;
		}
    };
}

#endif /* triangle3_h */
