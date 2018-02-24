#ifndef SPHERE_H
#define SPHERE_H

#include "bounding_box.h"

namespace motu {
	struct Sphere {
		Vector3 centre;
		float radius;

		Sphere(){}

		Sphere(const Vector3 &centre, float radius) : centre(centre), radius(radius) {}

		bool contains(const Vector3 &vec3) const{
			return (vec3 - centre).squareMagnitude() <= (radius * radius);
		}

		BoundingBox bounds() const {
			return BoundingBox(centre.x - radius, centre.y - radius, centre.z - radius, centre.x + radius, centre.y + radius, centre.z + radius);
		}

		bool operator==(const Sphere &other) const {
			return centre == other.centre && radius == other.radius;
		}
	};
}

#endif