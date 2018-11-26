//
//  spline.h
//  World Maker
//
//  Created by Jerome Johnson on 3/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef spline_h
#define spline_h

#include "vector3.h"
#include "edge.h"
#include "plane.h"

namespace motu{
    struct Spline{
        Vector3 endA, endB;
        
        Spline(){}
        
        Spline(const Vector3 &endA, const Vector3 &endB) : endA(endA), endB(endB){}
        
        Edge edge() const{
            return Edge(endA.x, endA.y, endB.x, endB.y);
        }

		Vector3 direction() const{
			return endB - endA;
		}

		Spline pair() const{
			return Spline(endB, endA);
		}

		Spline operator + (const Vector3 &vec) const {
			return Spline(endA + vec, endB + vec);
		}

		bool intersects(const Plane &plane, Vector3 &at) const {
			Vector3 dir(direction());
			float t;
			if (plane.intersectionTime(endA, dir, t)) {
				if (t >= 0.0f && t <= 1.0f) {
					at = endA + (dir * t);
					return true;
				}
			}
			return false;
		}

		bool intersects(const Plane &plane) const {
			Vector3 dir(direction());
			float t;
			if (plane.intersectionTime(endA, dir, t)) {
				return t >= 0.0f && t <= 1.0f;
			}
			return false;
		}

		bool operator < (const Spline spline) const {
			if (endA < spline.endA) {
				return true;
			}
			if (spline.endA < endA) {
				return false;
			}
			return endB < spline.endB;
		}

		bool operator == (const Spline &spline) const{
			return endA == spline.endA && endB == spline.endB;
		}

		bool operator != (const Spline &spline) const {
			return endA != spline.endA || endB != spline.endB;
		}

		size_t hash() const {
			return (endA.hash() * 7) ^ (endB.hash() * 3);
		}
    };

	struct SplineWithNormals : public Spline {
		Vector3 normalA, normalB;

		SplineWithNormals() {}

		SplineWithNormals(const Vector3 &endA, const Vector3 &endB, const Vector3 &normalA, const Vector3 &normalB) : Spline(endA, endB), normalA(normalA), normalB(normalB) {
		}

		bool intersects(const Plane &plane, Vector3 &at, Vector3 &normal) const{
			Vector3 dir(direction());
			float t;
			if (plane.intersectionTime(endA, dir, t)) {
				if (t >= 0.0f && t <= 1.0f) {
					at = endA + (dir * t);
					normal = (normalA * (1.0f - t)) + (normalB * t);
					return true;
				}
			}
			return false;
		}
	};

	struct SplineWithNormalsAndUV : public SplineWithNormals {
		Vector2 uvA, uvB;

		SplineWithNormalsAndUV() {}

		SplineWithNormalsAndUV(
			const Vector3 &endA, const Vector3 &endB,
			const Vector3 &normalA, const Vector3 &normalB,
			const Vector2 &uvA, const Vector2 &uvB) : SplineWithNormals(endA, endB, normalA, normalB),
			uvA(uvA), uvB(uvB) {}

		bool intersects(const Plane &plane, Vector3 &at, Vector3 &normal, Vector2 &uv) const {
			Vector3 dir(direction());
			float t;
			if (plane.intersectionTime(endA, dir, t)) {
				if (t >= 0.0f && t <= 1.0f) {
					at = endA + (dir * t);
					normal = (normalA * (1.0f - t)) + (normalB * t);
					uv = (uvA * (1.0f - t)) + (uvB * t);
					return true;
				}
			}
			return false;
		}
	};
}

#endif /* spline_h */
