#include "mesh_triangle_map.h"
#include "mesh.h"

using namespace motu;

MeshTriangleMap::MeshTriangleMap(const Mesh &mesh) {
	auto vertices = new std::pair<int, int>[mesh.vertices.size()];
	auto triangleBuffer = new int[mesh.triangles.size()];
	memset(vertices, 0, mesh.vertices.size() * sizeof(std::pair<int, int>));
	for (size_t i = 0; i != mesh.triangles.size(); ++i) {
		++vertices[mesh.triangles[i]].second;
	}
	int offset = 0;
	for (size_t i = 0; i != mesh.vertices.size(); ++i) {
		vertices[i].first = offset;
		offset += vertices[i].second;
		vertices[i].second = vertices[i].first;
	}
	for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
		for (size_t j = -2; j != 1; ++j) {
			triangleBuffer[vertices[mesh.triangles[i + j]].second++] = static_cast<int>(i - 2);
		}
	}
	this->vertices = std::unique_ptr<std::pair<int, int>>(vertices);
	this->triangleBuffer = std::unique_ptr<int>(triangleBuffer);
}