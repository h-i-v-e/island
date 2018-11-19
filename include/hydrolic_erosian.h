#ifndef HYDROLIC_EROSIAN_H
#define HYDROLIC_EROSIAN_H

#include <unordered_set>
#include <random>
#include <unordered_set>

#include "rivers.h"
#include "lake.h"
#include "decoration.h"

namespace motu {
	struct Mesh;
	class MeshEdgeMap;

	typedef std::unordered_set<int> RockSet;

	Mesh &applyHydrolicErosian(std::default_random_engine &rd, const RockSet &rock, Decoration &decoration);

	Mesh &applyHydrolicErosian(Mesh &mesh, const RockSet &rock, bool includeSea = false);
}

#endif