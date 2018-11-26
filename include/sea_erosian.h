#ifndef MOTU_SEA_EROSIAN
#define MOTU_SEA_EROSIAN

#include <vector>
#include <memory>
#include <random>
#include <unordered_set>

namespace motu {

	struct Mesh;
	struct Decoration;
	class MeshEdgeMap;

	typedef std::vector<std::unique_ptr<std::vector<std::pair<int, int>>>> Coastlines;

	void applySeaErosian(Mesh &mesh, const MeshEdgeMap &, float strength);

	void improveCliffs(Mesh &mesh, const MeshEdgeMap &, std::unordered_set<int> &cliffs);

	void mapCoastlines(const Mesh &, const MeshEdgeMap &, Coastlines &);

	void smoothCoastlines(Mesh &, const MeshEdgeMap &);

	void eatCoastlines(Mesh &, const MeshEdgeMap &, int steps);

	void formBeaches(Mesh &, const MeshEdgeMap &);

	void placeCoastalRocks(std::default_random_engine &, Decoration &decoration);
}

#endif