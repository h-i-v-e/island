#ifndef MOTU_MESH_TESSELATOR
#define MOTU_MESH_TESSELATOR

namespace motu {
	struct Mesh;
	struct MeshWithUV;

	Mesh &tesselate(Mesh &mesh);

	Mesh &tesselate(Mesh &mesh, float greaterThan);

	MeshWithUV &tesselate(MeshWithUV &mesh);
}

#endif