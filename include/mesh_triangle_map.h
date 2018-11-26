#ifndef MESH_TRIANGLE_MAP_H
#define MESH_TRIANGLE_MAP_H

#include <utility>
#include <memory>

namespace motu {
	struct Mesh;

	class MeshTriangleMap {
	public:
		typedef std::pair<const int*, const int*> IteratorPair;

		MeshTriangleMap(const Mesh &mesh);

		IteratorPair vertex(int offset) const {
			const std::pair<int, int> &vert = vertices.get()[offset];
			return std::make_pair(triangleBuffer.get() + vert.first, triangleBuffer.get() + vert.second);
		}
	private:
		std::unique_ptr<int> triangleBuffer;
		std::unique_ptr<std::pair<int, int>> vertices;
	};
}

#endif