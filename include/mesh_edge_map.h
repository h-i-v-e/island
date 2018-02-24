#ifndef MESH_EDGE_MAP
#define MESH_EDGE_MAP

#include <utility>
#include <cstdint>

namespace motu {

	struct Mesh;

	class MeshEdgeMap
	{
	public:
		MeshEdgeMap(const Mesh &mesh);

		~MeshEdgeMap() {
			delete[] neighbourBuffer;
			delete[] vertices;
		}

		std::pair<const int*, const int*> vertex(int offset) const {
			const std::pair<int, int> &vert = vertices[offset];
			return std::make_pair(neighbourBuffer + vert.first, neighbourBuffer + vert.second);
		}
	private:
		int * neighbourBuffer;
		std::pair<int, int> *vertices;
	};
}

#endif