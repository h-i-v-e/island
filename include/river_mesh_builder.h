#ifndef MOTU_RIVER_MESH_BUILDER
#define MOTU_RIVER_MESH_BUILDER

#include <unordered_set>
#include <vector>

#include "rivers.h"

namespace motu {
	struct Mesh;
	struct MeshWithUV;
	class MeshEdgeMap;
	class MeshTriangleMap;
	class Decoration;

	MeshWithUV &createRiverMesh(float uvScale, Decoration &decoration, const Rivers::RiverList &rivers, Mesh &terrain, MeshWithUV &output, const MeshEdgeMap &mem, const MeshTriangleMap &mtm);
}

#endif