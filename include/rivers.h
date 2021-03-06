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
	class MeshTriangleMap;

	struct River {
		struct Vertex {
			int index, flow;
			float surface;

			Vertex() {}

			Vertex(int index, int flow) : index(index), flow(flow), surface(0.0f) {}
		};

		typedef std::vector<Vertex> Vertices;
		typedef std::shared_ptr<River> Ptr;

		Vertices vertices;
		int join;
	};

	class Rivers {
	public:
		typedef std::vector<River::Ptr> RiverList;

		Rivers(Mesh &mesh, const MeshEdgeMap &, float maxHeight, float flowSDthreshold);

		const RiverList &riverList() const{
			return mRivers;
		}

		void smooth(Mesh &mesh, const MeshEdgeMap &) const;

		void smooth(Mesh &mesh, const MeshEdgeMap &mem, int iterations) const{
			while (iterations--) {
				smooth(mesh, mem);
			}
		}

		void jiggle(Mesh &mesh) const;

		void carveInto(Mesh &mesh, const MeshEdgeMap &, bool formDeltas) const;

		Mesh &getMesh(const River &, const Mesh &, Mesh &) const;
	private:
		RiverList mRivers;
		float depthMultiplier, maxHeight;
	};
}

#endif