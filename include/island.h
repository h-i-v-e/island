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
#include "decoration.h"

namespace motu {

	class Island {
	public:
		struct RiverVertex {
			Vector3 vertex;
			int flow;

			RiverVertex(const Vector3 &vertex, int flow) : vertex(vertex), flow(flow) {}

			RiverVertex() {}
		};

		typedef Grid<uint32_t> NormalAndOcclusianMap;
		typedef Grid<Vector3> NormalMap;
		typedef Grid<uint8_t> OcclusianMap;
		typedef Grid<uint32_t> VegetationMap;
		typedef std::vector<std::shared_ptr<Mesh>> RiverMeshes;
		typedef std::vector<RiverVertex> VertexList;
		typedef std::vector<std::shared_ptr<VertexList>> RiverVertexLists;

		struct Options {
			float maxRiverGradient;
			float riverDepth;
			float riverSourceSDThreshold;
			float maxZ;
			float waterRatio;
			float slopeMultiplier;
			float coastalSlopeMultiplier;
			float noiseMultiplier;
			int erosianPasses;

			Options() : maxRiverGradient(0.05f), riverDepth(0.005f), riverSourceSDThreshold(3.0f), maxZ(0.2f), waterRatio(0.5f), slopeMultiplier(1.3f), coastalSlopeMultiplier(1.0f), noiseMultiplier(0.0005f), erosianPasses(16) {}
		};

		Island(std::default_random_engine &rnd, const Options &options) : maxZ(options.maxZ){
			generateTopology(rnd, options);
		}

		const Mesh &lod(int offset) const {
			return lods[offset];
		}

		/*const Rivers &rivers() const {
			return *mRivers;
		}*/

		const RiverMeshes &riverMeshes() const {
			return mRiverMeshes;
		}

		void generateVegetationMap(VegetationMap &) const;

		void generateNormalAndOcclusianMap(NormalAndOcclusianMap &) const;

		const Lake::Lakes lakes() const {
			return mLakes;
		}

		const RiverVertexLists &riverVertexLists() const{
			return mRiverVertexLists;
		}

		const Decoration &decoration() const{
			return mDecoration;
		}

		const Mesh &soilRichness() const {
			return mSoilRichness;
		}

	private:
		float maxZ, maxHeight;
		Mesh lods[3];
		Lake::Lakes mLakes;
		//std::unique_ptr<Rivers> mRivers;
		RiverMeshes mRiverMeshes;
		RiverVertexLists mRiverVertexLists;
		Decoration mDecoration;
		Mesh mSoilRichness;

		void generateTopology(std::default_random_engine &, const Options &);
	};
}

#endif