#ifndef MOTU_DECORATION
#define MOTU_DECORATION

#include <vector>
#include <random>
#include <unordered_set>

#include "vector2.h"
#include "grid.h"
#include "mesh.h"

namespace motu {

	struct Decoration {
		std::vector<Vector2> trees, bushes, bigRocks, mediumRocks, smallRocks, forestScatter;
		Mesh mesh;
		std::vector<float> forest, soilRichness;
		std::unordered_set<int> occupied;

		Decoration(const Mesh &topology) : mesh(topology) {}
		
		void addRocks(std::default_random_engine &rnd);
	};
}

#endif