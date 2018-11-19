#ifndef MOTU_MESH_TESSELATOR
#define MOTU_MESH_TESSELATOR

namespace motu {
	struct Mesh;

	Mesh &tesselate(Mesh &mesh);

	Mesh &tesselate(Mesh &mesh, float greaterThan);
}

#endif