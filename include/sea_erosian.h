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

	void applySeaErosian(Mesh &mesh, float strength);

	void improveCliffs(Mesh &mesh, std::unordered_set<int> &cliffs);

	void mapCoastlines(const Mesh &, Coastlines &);

	void smoothCoastlines(Mesh &);

	void eatCoastlines(Mesh &, int steps);

	void formBeaches(Mesh &);

	void placeCoastalRocks(std::default_random_engine &, Decoration &decoration);
}

#endif