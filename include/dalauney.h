#ifndef DALAUNEY_H
#define DALAUNEY_H

#include <vector>

namespace motu {
	struct Vector3;
	struct Mesh;

	Mesh &createDalauneyMesh(const std::vector<Vector3> &vertices, Mesh &out);
}

#endif