#ifndef MOTU_MESH_TESSELATOR
#define MOTU_MESH_TESSELATOR

#include <unordered_set>

namespace motu {
	struct Mesh;
	struct MeshWithUV;

	Mesh &tesselate(Mesh &mesh);

	Mesh &tesselate(Mesh &mesh, float greaterThan);

	Mesh &tesselate(Mesh &mesh, std::unordered_set<int> &in);

	MeshWithUV &tesselate(MeshWithUV &mesh);
}

#endif