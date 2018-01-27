#include "hydrolic_erosian.h"
#include "mesh.h"
#include "mesh_edge_map.h"

using namespace motu;

namespace {
	inline void trackErosian(Mesh &mesh, const MeshEdgeMap &edgeMap, int offset, float carryCapacity) {
		Vector3 *next = &mesh.vertices[offset];
		if (next->z <= 0.0f) {
			return;
		}
		float speed = 0.0f, carrying = 0.0f, lowest = next->z;
		while (true) {
			int down = -1;
			std::pair<const uint32_t *, const uint32_t *> neighbours(edgeMap.vertex(offset));
			while (neighbours.first != neighbours.second){
				const Vector3 &vert = mesh.vertices[*neighbours.first];
				if (vert.z < lowest) {
					down = *neighbours.first;
					lowest = vert.z;
				}
				++neighbours.first;
			}
			if (lowest < 0.0f) {
				return;
			}
			if (down == -1) {
				next->z += carrying;
				return;
			}
			speed += next->z - lowest;
			speed *= 0.5f;
			float capacity = speed * carryCapacity;
			next = &mesh.vertices[down];
			next->z += (capacity - carrying);
			carrying = capacity;
		}
	}
}

Mesh &motu::applyHydrolicErosian(Mesh &mesh, const MeshEdgeMap &edges, int drops) {
	float carryCapacity = 10.0f / mesh.vertices.size();
	while (drops--) {
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			trackErosian(mesh, edges, i, carryCapacity);
		}
	}
	return mesh;
}
