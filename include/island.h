#ifndef ISLAND_H
#define ISLAND_H

#ifndef MOTU_TEXTURE_MAP_SIZE
#define MOTU_TEXTURE_MAP_SIZE 2048
#endif

#include <random>

#include "grid.h"
#include "mesh.h"
#include "lake.h"
#include "rivers.h"

namespace motu {

	class Island {
	public:
		typedef Grid<uint32_t> NormalAndOcclusianMap;
		typedef Grid<Vector3> NormalMap;
		typedef Grid<uint8_t> OcclusianMap;
		typedef Grid<uint32_t> AlbedoMap;

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
			float riverSourceSDThreshold;
			float maxZ;
			float waterRatio;
			float slopeMultiplier;
			float coastalSlopeMultiplier;
			float noiseMultiplier;
			int erosianPasses;

			Options() : maxRiverGradient(0.01f), riverDepth(0.0025f), riverSourceSDThreshold(2.0f), maxZ(0.05f), waterRatio(0.5f), slopeMultiplier(1.1f), coastalSlopeMultiplier(1.0f), noiseMultiplier(0.0005), erosianPasses(16) {}
		};

		Island(std::default_random_engine &rnd, const Options &options) : maxZ(options.maxZ),
			mNormalAndOcclusianMap(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE),
			albedo(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE){
			generateTopology(rnd, options);
		}

		const Mesh &lod(int offset) const {
			return lods[offset];
		}

		const Rivers &rivers() const {
			return *mRivers;
		}

		const AlbedoMap &albedoMap() const {
			return albedo;
		}

		const NormalAndOcclusianMap &normalAndOcclusianMap() const {
			return mNormalAndOcclusianMap;
		}

		const Lake::Lakes lakes() const {
			return mLakes;
		}

	private:
		float maxZ, maxHeight;
		Mesh lods[3];
		NormalAndOcclusianMap mNormalAndOcclusianMap;
		AlbedoMap albedo;
		Lake::Lakes mLakes;
		std::unique_ptr<Rivers> mRivers;

		void generateTopology(std::default_random_engine &, const Options &);
	};
}

#endif