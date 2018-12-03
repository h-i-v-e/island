#include "mesh_edge_map.h"
#include "mesh.h"

using namespace motu;

MeshEdgeMap::MeshEdgeMap(const Mesh &mesh) {
	typedef std::pair<int, int> Edges;
	vertices = new Edges[mesh.vertices.size()];
	int total = 0;
	for (int i = 2; i < mesh.triangles.size(); i += 3) {
		for (int j = i - 2; j <= i; ++j) {
			for (int k = i - 2; k <= i; ++k) {
				if (j != k) {
					++vertices[mesh.triangles[k]].second;
					++total;
				}
			}
		}
	}
	neighbourBuffer = new int[total << 1];
	total = 0;
	for (int i = 0; i != mesh.vertices.size(); ++i) {
		vertices[i].first = total;
		total += (vertices[i].second << 1);
		vertices[i].second = vertices[i].first;
	}
	for (int i = 2; i < mesh.triangles.size(); i += 3) {
		for (int j = i - 2; j <= i; ++j) {
			for (int k = i - 2; k <= i; ++k) {
				if (j != k) {
					int vert = mesh.triangles[j];
					bool add = true;
					for (auto pair = vertices[mesh.triangles[k]]; pair.first != pair.second; ++pair.first) {
						if (neighbourBuffer[pair.first] == vert) {
							add = false;
							break;
						}
					}
					if (add) {
						neighbourBuffer[vertices[mesh.triangles[k]].second++] = vert;
					}
				}
			}
		}
	}
}