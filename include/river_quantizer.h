#include <queue>

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

	class RiverQuantiser {
	public:
		RiverQuantiser(const HeightMap &hm) : heightMap(hm), seaMap(hm.width(), hm.height()) {
			populateSeaMap();
		}

		QuantisedRiver &quantiseRiver(const Island::VertexList &, QuantisedRiver &);
	private:
		const HeightMap &heightMap;
		Grid<bool> seaMap;

		void populateSeaMap();
	};

	//rivers are deduped and the rivers they join are mapped with the same offsets in flowIntos
	void dedupeQuantisedRivers(HeightMap &grid, std::vector<QuantisedRiver*> &rivers, std::vector<int> &flowIntos);

	MeshWithUV &createQuantisedRiverMesh(HeightMap &, const QuantisedRiver &, MeshWithUV &mesh, float maxFlow);

	void joinQuantisedRiverMeshes(HeightMap &, MeshWithUV &from, MeshWithUV &to);
}