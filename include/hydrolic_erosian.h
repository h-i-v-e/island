#ifndef HYDROLIC_EROSIAN_H
#define HYDROLIC_EROSIAN_H

#include <unordered_set>
#include <random>

#include "rivers.h"
#include "lake.h"
#include "decoration.h"

namespace motu {
	struct Mesh;
	class MeshEdgeMap;

	Mesh &applyHydrolicErosian(std::default_random_engine &rd, Decoration &decoration);

	Mesh &applyHydrolicErosian(Mesh &mesh);
}

#endif