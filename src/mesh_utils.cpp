#include "mesh_utils.h"
#include "mesh.h"

using namespace motu;

Mesh &motu::smoothPeeks(Mesh &mesh, const MeshEdgeMap &mep) {
	std::vector<float> adjustments(mesh.vertices.size());
	size_t numVerts = mesh.vertices.size();
	for (size_t i = 0; i != numVerts; ++i) {
		float height = mesh.vertices[i].z;
		float total = height;
		int count = 1;
		bool max = true;
		for (auto j = mep.vertex(static_cast<int>(i)); j.first != j.second; ++j.first) {
			float h = mesh.vertices[*j.first].z;
			if (h >= height) {
				max = false;
				break;
			}
			total += h;
			++count;
		}
		if (max) {
			height = total / static_cast<float>(count);
		}
		adjustments[i] = height;
	}
	for (size_t i = 0; i != numVerts; ++i) {
		mesh.vertices[i].z = adjustments[i];
	}
	return mesh;
}