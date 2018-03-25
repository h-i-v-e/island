#ifndef HYDROLIC_EROSIAN_H
#define HYDROLIC_EROSIAN_H

#include <unordered_set>

#include "rivers.h"
#include "lake.h"

namespace motu {
	struct Mesh;
	class MeshEdgeMap;

	Mesh &applyHydrolicErosian(Mesh &mesh, const MeshEdgeMap &edgeMap, int iterations);
}

#endif