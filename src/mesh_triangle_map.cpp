#include "mesh_triangle_map.h"
#include "mesh.h"

using namespace motu;

MeshTriangleMap::MeshTriangleMap(const Mesh &mesh) {
	vertices = new std::pair<int, int>[mesh.vertices.size()];
	triangleBuffer = new int[mesh.triangles.size()];
	memset(vertices, 0, mesh.vertices.size() * sizeof(std::pair<int, int>));
	for (int i = 0; i < mesh.triangles.size(); ++i) {
		++vertices[mesh.triangles[i]].second;
	}
	int offset = 0;
	for (int i = 0; i != mesh.vertices.size(); ++i) {
		vertices[i].first = offset;
		offset += vertices[i].second;
		vertices[i].second = vertices[i].first;
	}
	for (int i = 2; i < mesh.triangles.size(); i += 3) {
		for (int j = -2; j != 1; ++j) {
			triangleBuffer[vertices[mesh.triangles[i + j]].second++] = i - 2;
		}
	}
}