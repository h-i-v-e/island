#include "mesh_edge_map.h"
#include "mesh.h"

using namespace motu;

MeshEdgeMap::MeshEdgeMap(const Mesh &mesh) {
	typedef std::pair<uint32_t, uint32_t> Edges;
	vertices = new Edges[mesh.vertices.size()];
	size_t total = 0;
	for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
		for (size_t j = i - 2; j <= i; ++j) {
			for (size_t k = i - 2; k <= i; ++k) {
				if (j != k) {
					++vertices[mesh.triangles[k]].second;
					++total;
				}
			}
		}
	}
	neighbourBuffer = new uint32_t[total];
	total = 0;
	for (size_t i = 0; i != mesh.vertices.size(); ++i) {
		vertices[i].first = total;
		total += vertices[i].second;
		vertices[i].second = vertices[i].first;
	}
	for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
		for (size_t j = i - 2; j <= i; ++j) {
			for (size_t k = i - 2; k <= i; ++k) {
				if (j != k) {
					neighbourBuffer[vertices[mesh.triangles[k]].second++] = mesh.triangles[j];
				}
			}
		}
	}
}