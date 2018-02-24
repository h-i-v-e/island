#ifndef ISLAND_H
#define ISLAND_H

#ifndef MOTU_TEXTURE_MAP_SIZE
#define MOTU_TEXTURE_MAP_SIZE 2048
#endif

#include <random>

#include "grid.h"
#include "mesh.h"

namespace motu {
	class Island {
	public:
		typedef Grid<uint32_t> NormalAndOcclusianMap;
		typedef Grid<Vector3> NormalMap;
		typedef Grid<uint8_t> OcclusianMap;
		typedef Grid<uint16_t> AlbedoMap;

		struct Pallete {
			Vector3 sand, cliff, grass, mountain;

			static Vector3 white() {
				return Vector3(1.0f, 1.0f, 1.0f);
			}

			Pallete() : sand(white()), cliff(white()), grass(white()), mountain(white()) {}
		};

		struct Options {
			Pallete pallete;
			float maxRiverGradient;
			float riverDepth;
			float maxZ;
			float waterRatio;
			float slopeMultiplier;
			int erosianPasses;

			Options() : maxRiverGradient(0.01f), riverDepth(0.0015), maxZ(0.2f), waterRatio(0.5f), slopeMultiplier(1.1f), erosianPasses(16) {}
		};

		Island(std::default_random_engine &rnd, const Options &options) : maxZ(options.maxZ),
			mNormalAndOcclusianMap(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE),
			albedo(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE){
			generateTopology(rnd, options);
		}

		const Mesh &lod(int offset) const {
			return lods[offset];
		}

		const Mesh &rivers() const {
			return mRivers;
		}

		const AlbedoMap &albedoMap() const {
			return albedo;
		}

		const NormalAndOcclusianMap &normalAndOcclusianMap() const {
			return mNormalAndOcclusianMap;
		}

	private:
		float maxZ, maxHeight;
		Mesh lods[3], mRivers;
		NormalAndOcclusianMap mNormalAndOcclusianMap;
		AlbedoMap albedo;

		void generateTopology(std::default_random_engine &, const Options &);
	};
}

#endif