#ifndef RIVERS_H
#define RIVERS_H

#include <utility>
#include <cstdint>
#include <vector>

namespace motu {
	struct Mesh;
	class MeshEdgeMap;

	class Rivers {
	public:
		typedef std::vector<std::pair<int, int>> River;

		Rivers(const Mesh &mesh, const MeshEdgeMap &);

		Rivers(const Rivers &rivers, const Mesh &old, const Mesh &nw, const MeshEdgeMap &);

		const std::vector<River> &rivers() const{
			return mRivers;
		}

		void smooth(Mesh &mesh, const MeshEdgeMap &) const;

		void carveInto(Mesh &mesh, const MeshEdgeMap &, float depthMultiplier) const;

	private:
		std::vector<River> mRivers;
	};
}

#endif