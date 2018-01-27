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

		std::pair<const uint32_t*, const uint32_t*> vertex(uint32_t offset) const {
			const std::pair<uint32_t, uint32_t> &vert = vertices[offset];
			return std::make_pair(neighbourBuffer + vert.first, neighbourBuffer + vert.second);
		}
	private:
		uint32_t * neighbourBuffer;
		std::pair<uint32_t, uint32_t> *vertices;
	};
}

#endif