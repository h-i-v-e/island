#ifndef MOTU_SEA_EROSIAN
#define MOTU_SEA_EROSIAN

#include <vector>
#include <memory>

namespace motu {

	struct Mesh;
	class MeshEdgeMap;

	typedef std::vector<std::unique_ptr<std::vector<std::pair<int, int>>>> Coastlines;

	void applySeaErosian(Mesh &mesh);

	void applySeaErosian(const MeshEdgeMap &, Mesh &mesh);

	void mapCoastlines(const MeshEdgeMap &, const Mesh &, Coastlines &);
}

#endif