#ifndef MOTU_TREE_BILLBOARDS
#define MOTU_TREE_BILLBOARDS

#include <cmath>

#include "mesh.h"

namespace motu {
	struct TreeBillboards : public Mesh{
		struct PositionScale {
			Vector3 position;
			float scale;

			PositionScale(const Vector3 &pos, float scale) : position(pos), scale(scale) {}

			PositionScale() {}
		};

		typedef std::vector<PositionScale> Positions;
		typedef std::unique_ptr<TreeBillboards> UPtr;
		typedef std::vector<int> Offsets;
		typedef std::vector<std::unique_ptr<Offsets>> OctantOffsets;

		TreeBillboards(const Positions &positions);

		static std::vector<UPtr> &createOctants(const Positions &positions, std::vector<UPtr> &octants, OctantOffsets &octantOffsets);
	};
}

#endif