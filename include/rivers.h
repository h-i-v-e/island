#ifndef RIVERS_H
#define RIVERS_H

#include <utility>
#include <cstdint>
#include <vector>
#include <memory>
#include <unordered_set>

#include "lake.h"

namespace motu {
	struct Mesh;
	class MeshEdgeMap;
	struct MeshTriangleMap;

	struct River {
		typedef std::vector<std::pair<int, int>> Vertices;
		typedef std::shared_ptr<River> Ptr;

		Vertices vertices;
		int join;
	};

	class Rivers {
	public:
		typedef std::vector<River::Ptr> RiverList;

		Rivers(Mesh &mesh, const MeshEdgeMap &, Lake::Lakes &lakes, float flowSDthreshold = 3.0f);

		const RiverList &riverList() const{
			return mRivers;
		}

		void smooth(Mesh &mesh, const MeshEdgeMap &) const;

		void carveInto(Mesh &mesh, const MeshEdgeMap &, float depthMultiplier, float maxGradient) const;

		Mesh &getMesh(const River &, Mesh &, const MeshTriangleMap &);
	private:
		RiverList mRivers;
	};
}

#endif