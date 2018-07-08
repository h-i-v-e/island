#ifndef MOTU_DECORATION
#define MOTU_DECORATION

#include <vector>
#include <random>

#include "vector2.h"

namespace motu {
	struct Mesh;

	struct Decoration {
		std::vector<Vector2> trees, bushes, bigRocks, mediumRocks, smallRocks;

		void addRocks(std::default_random_engine &rnd, const Mesh &mesh);
	};
}

#endif