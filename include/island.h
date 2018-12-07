#ifndef ISLAND_H
#define ISLAND_H

#include <random>
#include <iostream>

#include "grid.h"
#include "mesh.h"
#include "lake.h"
#include "rivers.h"
#include "decoration.h"
#include "colour.h"
#include "raster.h"

namespace motu {

	class Raster;

	class Island {
	public:
		struct RiverVertex {
			Vector3 vertex;
			int flow;

			RiverVertex(const Vector3 &vertex, int flow) : vertex(vertex), flow(flow) {}

			RiverVertex() {}
		};

		typedef Grid<uint32_t> Image;
		typedef Grid<Colour24> Image24;
		typedef Grid<Vector3> NormalMap;
		typedef Grid<uint8_t> OcclusianMap;
		typedef Grid<uint32_t> VegetationMap;
		typedef std::vector<std::unique_ptr<Mesh>> RiverMeshes;

		struct Options {
			float maxZ;
			float waterRatio;
			float slopeMultiplier;
			float coastalSlopeMultiplier;
			float noiseMultiplier;

			Options() : maxZ(0.2f), waterRatio(0.5f), slopeMultiplier(1.3f), coastalSlopeMultiplier(1.0f), noiseMultiplier(0.0005f) {}
		};

		Island(std::default_random_engine &rnd, const Options &options) : maxZ(options.maxZ){
			generateTopology(rnd, options);
		}

		Island(){}

		const Mesh &lod(int offset) const {
			return lods[offset];
		}

		const MeshWithUV &riverMesh() const {
			return mRiverMesh;
		}

		void generateNormalAndOcclusianMap(Image &) const;

		void generateNormalMap(int lod, Image24 &) const;

		const Decoration &decoration() const{
			return mDecoration;
		}

		static constexpr float size() {
			return 8192.0f;
		}

		friend std::ostream& operator<<(std::ostream &, const Island &);

		friend std::istream& operator>>(std::istream &, Island &);

	private:
		float maxZ, maxHeight;
		Mesh lods[3];
		MeshWithUV mRiverMesh;
		Decoration mDecoration;

		void generateTopology(std::default_random_engine &, const Options &);
	};
}

#endif