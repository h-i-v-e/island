#ifndef MOTU_DECORATION
#define MOTU_DECORATION

#include <vector>
#include <random>
#include <unordered_set>

#include "vector2.h"
#include "grid.h"
#include <memory>

#include "mesh.h"
#include "tree_billboards.h"

namespace motu {

	struct Decoration {
		//std::vector<Vector3> bushes, bigRocks, mediumRocks, smallRocks, forestScatter, treePositions;
		std::vector<int> trees, bushes, rocks;
		Mesh mesh;
		std::vector<float> forest, soilRichness;
		std::unordered_set<int> occupied;

		Decoration() {}

		Decoration(const Mesh &topology) : mesh(topology) {}

		void computeZValues(const Mesh &mesh);
		
		void addRocks(const MeshEdgeMap &myEdges, std::default_random_engine &rnd);

		std::vector<Vector3> &getTrees(const Mesh &lod0, const MeshEdgeMap &mem, std::vector<Vector3> &) const;

		std::vector<Vector3> &getBushes(const Mesh &lod0, std::vector<Vector3> &) const;

		std::vector<Vector3> &getRocks(const Mesh &lod0, const MeshEdgeMap &mem, std::vector<Vector3> &) const;

		friend std::ostream& operator<<(std::ostream &, const Decoration &);

		friend std::istream& operator>>(std::istream &, Decoration &);
	};
}

#endif