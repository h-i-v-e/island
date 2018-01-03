#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include "vector3.h"
#include "triangle3.h"

namespace motu {
	struct BoundingBox {
		Vector3 upper, lower;

		BoundingBox() {}

		BoundingBox(const Vector3 &upper, const Vector3 &lower) : upper(upper), lower(lower) {}

		BoundingBox(float xa, float ya, float za, float xb, float yb, float zb) : upper(xa, ya, za), lower(xb, yb, zb) {}

		static constexpr bool between(float a, float b, float c) {
			return (c >= (a - FLT_EPSILON) && c <= (b + FLT_EPSILON)) ||
				(c <= (a + FLT_EPSILON) && c >= (b - FLT_EPSILON));
		}

		void getPlanes(Plane *planes) const {
			for (size_t i = 0; i != 3; ++i) {
				planes[i].point = lower;
				planes[i + 3].point = upper;
			}
			for (size_t i = 0; i != 6; i += 3) {
				planes[i].normal = Vector3::unitX();
				planes[i + 1].normal = Vector3::unitY();
				planes[i + 2].normal = Vector3::unitZ();
			}
		}

		bool contains(const Vector3 &vec) const {
			return between(lower.x, upper.x, vec.x) &&
				between(lower.y, upper.y, vec.y) &&
				between(lower.z, upper.z, vec.z);
		}

		bool intersects(const Triangle3 &t3) const {
			for (size_t i = 0; i != 3; ++i) {
				if (contains(t3.vertices[i])) {
					return true;
				}
			}
			Spline splines[3];
			t3.getSplines(splines);
			Plane planes[6];
			getPlanes(planes);
			for (size_t i = 0; i != 6; ++i) {
				for (size_t j = 0; j != 3; ++j) {
					if (splines[i].intersects(planes[j])) {
						return true;
					}
				}
			}
			return false;
		}

		bool contains(const Triangle3 &t3) const {
			for (size_t i = 0; i != 3; ++i) {
				if (!contains(t3.vertices[i])) {
					return false;
				}
			}
			return true;
		}
	};
}

#endif