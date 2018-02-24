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
	neighbourBuffer = new int[total];
	total = 0;
	for (int i = 0; i != mesh.vertices.size(); ++i) {
		vertices[i].first = total;
		total += vertices[i].second;
		vertices[i].second = vertices[i].first;
	}
	for (int i = 2; i < mesh.triangles.size(); i += 3) {
		for (int j = i - 2; j <= i; ++j) {
			for (int k = i - 2; k <= i; ++k) {
				if (j != k) {
					neighbourBuffer[vertices[mesh.triangles[k]].second++] = mesh.triangles[j];
				}
			}
		}
	}
}