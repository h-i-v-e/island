#ifndef MOTU_SEA_EROSIAN
#define MOTU_SEA_EROSIAN

#include <vector>
#include <memory>
#include <random>

namespace motu {

	struct Mesh;
	struct Decoration;
	class MeshEdgeMap;

	typedef std::vector<std::unique_ptr<std::vector<std::pair<int, int>>>> Coastlines;

	void applySeaErosian(Mesh &mesh, float strength);

	void applySeaErosian(Mesh &mesh, float strength);

	void improveCliffs(Mesh &mesh);

	void improveCliffs(Mesh &mesh);

	void mapCoastlines(const Mesh &, Coastlines &);

	void smoothCoastlines(Mesh &);

	void eatCoastlines(Mesh &);

	void formBeaches(Mesh &);

	void placeCoastalRocks(std::default_random_engine &, Mesh &, Decoration &decoration);
}

#endif