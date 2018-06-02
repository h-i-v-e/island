#ifndef MOTU_SEA_EROSIAN
#define MOTU_SEA_EROSIAN

#include <vector>
#include <memory>

namespace motu {

	struct Mesh;
	class MeshEdgeMap;

	typedef std::vector<std::unique_ptr<std::vector<std::pair<int, int>>>> Coastlines;

	void applySeaErosian(Mesh &mesh, float strength);

	void applySeaErosian(const MeshEdgeMap &, Mesh &mesh, float strength);

	void improveCliffs(Mesh &mesh);

	void improveCliffs(const MeshEdgeMap &, Mesh &mesh);

	void mapCoastlines(const MeshEdgeMap &, const Mesh &, Coastlines &);

	void smoothCoastlines(const MeshEdgeMap &, Mesh &);

	void eatCoastlines(const MeshEdgeMap &, Mesh &);

	void formBeaches(const MeshEdgeMap &, Mesh &);
}

#endif