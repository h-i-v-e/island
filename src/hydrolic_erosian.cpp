#include "hydrolic_erosian.h"
#include "mesh.h"
#include "mesh_edge_map.h"
#include "rivers.h"
#include "sea_erosian.h"

#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <stack>

using namespace motu;

namespace {

	void trackErosian(Mesh &mesh, const MeshEdgeMap &edgeMap, int offset, float carrying, float carryCapacity) {
		Vector3 *next = &mesh.vertices[offset];
		if (next->z < 0.0f) {
			return;
		}
		float speed = carrying / carryCapacity, lowest = next->z;
		while (true) {
			int down = -1;
			std::pair<const int *, const int *> neighbours(edgeMap.vertex(offset));
			while (neighbours.first != neighbours.second){
				const Vector3 &vert = mesh.vertices[*neighbours.first];
				if (vert.z < lowest) {
					down = *neighbours.first;
					lowest = vert.z;
				}
				++neighbours.first;
			}
			if (lowest < 0.0f) {
				//next->z += carrying;
				return;
			}
			if (down == -1) {
				next->z += carrying;
				return;
			}
			speed += next->z - lowest;
			speed *= 0.5f;
			//speed = next->z - lowest;
			float capacity = (speed * speed * speed) * carryCapacity;
			next = &mesh.vertices[down];
			next->z += (capacity - carrying);
			carrying = capacity;
		}
	}

	/*struct EmptyOutflow {
		bool operator()(int vert, float flow, float z) const{
			return z < 0.0f;
		}

		void resolve(Mesh &mesh, const MeshEdgeMap &edgeMap) {}
	};

	struct ReallyEmptyOutflow {
		bool operator()(int vert, float flow, float z) const{
			return false;
		}

		void resolve(Mesh &mesh, const MeshEdgeMap &edgeMap) {}
	};

	void trackOutflow(int in, float carrying, float carryCapacity, Mesh &mesh, const MeshEdgeMap &edgeMap) {
		float z = mesh.vertices[in].z;
		while (z > 0.0f) {
			auto n = edgeMap.vertex(in);
			int lowest = -1;
			while (n.first != n.second) {
				int next = *n.first++;
				float nz = mesh.vertices[next].z;
				if (nz < z) {
					lowest = next;
					z = nz;
				}
			}
			if (lowest == -1) {
				mesh.vertices[in].z += carrying;
				return;
			}
		}
		trackErosian(mesh, edgeMap, in, carrying, carryCapacity, ReallyEmptyOutflow());
	}*/

	Mesh &erode(std::default_random_engine &rnd, Mesh &mesh, const MeshEdgeMap &edges, int drops) {
		std::uniform_int_distribution<int> dist(0, (mesh.triangles.size() / 3) - 1);
		float carryCapacity = 50.0f / mesh.vertices.size();
		while (drops--) {
			for (int i = 0; i != mesh.triangles.size(); ++i) {
				int offset = dist(rnd) * 3;
				int lowest = offset;
				float z = mesh.vertices[mesh.triangles[offset++]].z;
				for (int j = offset + 2; offset != j; ++offset) {
					if (mesh.vertices[mesh.triangles[offset]].z < z) {
						lowest = offset;
						z = mesh.vertices[mesh.triangles[offset]].z;
					}
				}
				trackErosian(mesh, edges, mesh.triangles[lowest], 0.0f, carryCapacity);
			}
		}
		//applySeaErosian(edges, mesh, carryCapacity);
		return mesh;
	}
}

Mesh &motu::applyHydrolicErosian(std::default_random_engine &rnd, Mesh &mesh, const MeshEdgeMap &edges, int drops) {
	return erode(rnd, mesh, edges, drops);
}