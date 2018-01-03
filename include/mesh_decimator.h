#ifndef MESH_DECIMATOR_H
#define MESH_DECIMATOR_H

#include <limits>

namespace motu {
	struct Mesh;

	int decimate(Mesh &mesh, int targetVertices = std::numeric_limits<int>::max(), bool decimateAllFree = true, float minAngle = 15.0);
}

#endif