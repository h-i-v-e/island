#include "island.h"
#include "vector2int.h"

namespace motu {
	struct QuantisedRiverNode : Vector2Int{
		int flow;

		QuantisedRiverNode(int x, int y, int flow) : Vector2Int(x, y), flow(flow) {}

		QuantisedRiverNode() {}
	};

	typedef std::vector<QuantisedRiverNode> QuantisedRiver;

	QuantisedRiver &quantiseRiver(const Grid<float> &, const Island::VertexList &, QuantisedRiver &);

	void dedupeQuantisedRivers(std::vector<QuantisedRiver*> &);

	MeshWithUV &createQuantisedRiverMesh(Grid<float> &, const QuantisedRiver &, MeshWithUV &mesh);
}