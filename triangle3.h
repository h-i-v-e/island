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

namespace worldmaker{
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
        
        Vector3 normal() const{
            return (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);
        }
        
        void getSplines(Spline *splines) const{
            splines[0] = Spline(vertices[0], vertices[1]);
            splines[1] = Spline(vertices[1], vertices[2]);
            splines[2] = Spline(vertices[2], vertices[0]);
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
    };
    
    struct Triangle3WithNormals : public Triangle3{
        Vector3 normals[3];
        
        Triangle3WithNormals(){}
        
        Triangle3WithNormals(const Triangle3 &verts) : Triangle3(verts){}
    };
}

#endif /* triangle3_h */
