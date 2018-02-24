#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <limits>

#include "triangle3.h"

namespace motu {
	struct BoundingBox {
		typedef Vector3 VectorType;

		Vector3 upper, lower;

		BoundingBox() {}

		BoundingBox(const Vector3 &upper, const Vector3 &lower) : upper(upper), lower(lower) {}

		BoundingBox(float xa, float ya, float za, float xb, float yb, float zb) : upper(xa, ya, za), lower(xb, yb, zb) {}

		static constexpr bool between(float a, float b, float c) {
			return (c >= (a - FLT_EPSILON) && c <= (b + FLT_EPSILON)) ||
				(c <= (a + FLT_EPSILON) && c >= (b - FLT_EPSILON));
		}

		void clear() {
			upper.x = upper.y = upper.z = std::numeric_limits<float>::max();
			lower.x = lower.y = lower.z = std::numeric_limits<float>::min();
		}

		BoundingBox &operator+= (const Vector3 &pt) {
			if (pt.x < upper.x) {
				upper.x = pt.x;
			}
			if (pt.x > lower.x) {
				lower.x = pt.x;
			}
			if (pt.y < upper.y) {
				upper.y = pt.y;
			}
			if (pt.y > lower.y) {
				lower.y = pt.y;
			}
			if (pt.z < upper.z) {
				upper.z = pt.z;
			}
			if (pt.z > lower.z) {
				lower.z = pt.z;
			}
		}

		BoundingBox operator + (const BoundingBox &box) const {
			return BoundingBox(
				std::min(upper.x, box.upper.x), std::min(upper.y, box.upper.y), std::min(upper.z, box.upper.z),
				std::max(lower.x, box.lower.x), std::max(lower.y, box.lower.y), std::max(lower.z, box.lower.z)
			);
		}

		constexpr float width() const {
			return lower.x - upper.x;
		}

		constexpr float height() const {
			return lower.y - upper.y;
		}

		constexpr float depth() const {
			return lower.z - upper.z;
		}

		float area() const {
			return width() * height() * depth();
		}

		void getPlanes(Plane *planes) const {
			for (int i = 0; i != 3; ++i) {
				planes[i].point = lower;
				planes[i + 3].point = upper;
			}
			for (int i = 0; i != 6; i += 3) {
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
			for (int i = 0; i != 3; ++i) {
				if (contains(t3.vertices[i])) {
					return true;
				}
			}
			Spline splines[3];
			t3.getSplines(splines);
			Plane planes[6];
			getPlanes(planes);
			for (int i = 0; i != 6; ++i) {
				for (int j = 0; j != 3; ++j) {
					if (splines[i].intersects(planes[j])) {
						return true;
					}
				}
			}
			return false;
		}

		bool intersects(const BoundingBox &other) const{
			return contains(other.lower) || contains(other.upper) ||
				other.contains(lower) || other.contains(upper);
		}

		bool contains(const Triangle3 &t3) const {
			for (int i = 0; i != 3; ++i) {
				if (!contains(t3.vertices[i])) {
					return false;
				}
			}
			return true;
		}
	};
}

#endif