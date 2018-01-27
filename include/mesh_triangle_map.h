#ifndef MESH_TRIANGLE_MAP_H
#define MESH_TRIANGLE_MAP_H

#include <utility>

namespace motu {
	struct Mesh;

	class MeshTriangleMap {
	public:
		MeshTriangleMap(const Mesh &mesh);

		~MeshTriangleMap() {
			delete[] triangleBuffer;
			delete[] vertices;
		}

		std::pair<const int*, const int*> vertex(int offset) const {
			const std::pair<int, int> &vert = vertices[offset];
			return std::make_pair(triangleBuffer + vert.first, triangleBuffer + vert.second);
		}
	private:
		int *triangleBuffer;
		std::pair<int, int> *vertices;
	};
}

#endif