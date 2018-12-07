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
	class Decoration;

	Mesh &applyHydrolicErosian(std::default_random_engine &rd, Mesh &mesh, const MeshEdgeMap &mem, Decoration &decoration);

	Mesh &applyHydrolicErosian(Mesh &mesh, const MeshEdgeMap &mem, const Decoration &decoration, bool includeSea = false);
}

#endif