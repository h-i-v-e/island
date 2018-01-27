#ifndef HYDROLIC_EROSIAN_H
#define HYDROLIC_EROSIAN_H

namespace motu {
	class Mesh;
	class MeshEdgeMap;

	Mesh &applyHydrolicErosian(Mesh &mesh, const MeshEdgeMap &edgeMap, int iterations);
}

#endif