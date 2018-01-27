#ifndef ISLAND_H
#define ISLAND_H

#include <random>

#include "grid.h"
#include "mesh.h"

namespace motu {
	class Island {
	public:
		typedef Grid<uint32_t> NormalAndOcclusianMap;

		Island(std::default_random_engine &rnd, float waterRatio, float maxZ) : maxZ(maxZ), mNormalAndOcclusianMap(2048, 2048) {
			generateTopology(rnd, waterRatio);
		}

		const Mesh &lod(size_t offset) const {
			return lods[offset];
		}

		const Mesh &rivers() const {
			return mRivers;
		}

		const NormalAndOcclusianMap &normalAndOcclusianMap() const {
			return mNormalAndOcclusianMap;
		}

	private:
		float maxZ, maxHeight;
		Mesh lods[3], mRivers;
		NormalAndOcclusianMap mNormalAndOcclusianMap;

		void generateTopology(std::default_random_engine &, float waterRatio);
	};
}

#endif