#ifndef MOTU_RIVER_MESH_BUILDER
#define MOTU_RIVER_MESH_BUILDER

#include <unordered_set>
#include <vector>

#include "rivers.h"

namespace motu {
	struct Mesh;
	class MeshEdgeMap;
	class MeshTriangleMap;

	class RiverMeshBuilder {
	public:
		RiverMeshBuilder(float flowMul, Mesh &terrain, Mesh &output, const MeshEdgeMap &mem, const MeshTriangleMap &mtm) :
			flowMul(flowMul), terrain(terrain), output(output), mem(mem), mtm(mtm){}

		void addRiver(const River &river);

		void smoothRiverBeds(const Rivers::RiverList &rivers);

	protected:
		typedef std::unordered_set<int> ISet;
		typedef std::unordered_map<int, int> IMap;

		float flowMul;
		Mesh &terrain, &output;
		const MeshEdgeMap &mem;
		const MeshTriangleMap &mtm;
		IMap addedVerts;
		ISet addedTriangles, added, waterfalls, leftVerts, rightVerts;
		std::vector<float> flows;
	};
}

#endif