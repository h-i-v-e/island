#include "island.h"
#include "vector2int.h"
#include "height_map.h"

namespace motu {
	struct QuantisedRiverNode : Vector2Int{
		int flow;

		QuantisedRiverNode(int x, int y, int flow) : Vector2Int(x, y), flow(flow) {}

		QuantisedRiverNode() {}
	};

	typedef std::vector<QuantisedRiverNode> QuantisedRiver;

	QuantisedRiver &quantiseRiver(const Grid<float> &, const Island::VertexList &, QuantisedRiver &);

	//rivers are deduped and the rivers they join are mapped with the same offsets in flowIntos
	void dedupeQuantisedRivers(const HeightMap &grid, std::vector<QuantisedRiver*> &rivers, std::vector<int> &flowIntos);

	MeshWithUV &createQuantisedRiverMesh(HeightMap &, const QuantisedRiver &, MeshWithUV &mesh, float seaLevel, float maxFlow);

	void joinQuantisedRiverMeshes(HeightMap &, MeshWithUV &from, MeshWithUV &to);
}