#ifndef MOTU_DECORATION
#define MOTU_DECORATION

#include <vector>
#include <random>
#include <unordered_set>

#include "vector2.h"
#include "grid.h"
#include <memory>

#include "mesh.h"
#include "tree_billboards.h"

namespace motu {

	class Decoration {
	public:
		const std::vector<Vector3> &trees() const {
			return treePositions;
		}

		const std::vector<Vector3> &bushes() const {
			return bushPositions;
		}

		const std::vector<Vector3> &rocks() const {
			return rockPositions;
		}

		friend std::ostream& operator<<(std::ostream &, const Decoration &);

		friend std::istream& operator>>(std::istream &, Decoration &);

		void addRock(int pos) {
			add(pos, mRocks);
		}

		bool isRock(int pos) const{
			return mRocks.find(pos) != mRocks.end();
		}

		void addBush(int pos) {
			add(pos, mBushes);
		}

		void addTree(int pos) {
			add(pos, mTrees);
		}

		void setRiver(int pos, bool addRock) {
			mRiver.insert(pos);
			if (occupied.find(pos) != occupied.end()) {
				mTrees.erase(pos);
				mBushes.erase(pos);
				if (addRock) {
					mRocks.insert(pos);
				}
			}
		}

		void reserveSoilRichness(size_t size) {
			soilRichness.resize(size, 0.0f);
		}

		void addSoilRichness(int i, float val) {
			soilRichness[i] += val;
		}

		float soilRichnessAt(int i) const{
			return soilRichness[i];
		}

		//erosian can me trees end up under water
		void removeUnderWaterTrees(const Mesh &mesh);

		void spreadSoilRichness(const Mesh &mesh, const MeshEdgeMap &mep);

		void normaliseSoilRichness();

		void computePositions(const Mesh &lod0, const MeshEdgeMap &mem);

		Grid<float> &fillForestMap(const Mesh &mesh, Grid<float> &) const;

		Grid<float> &fillRockMap(const Mesh &mesh, Grid<float> &) const;

		Grid<float> &fillRiverMap(const Mesh &mesh, Grid<float> &) const;

		Grid<float> &fillSoilRichnessMap(const Mesh &mesh, Grid<float> &) const;

	private:
		void add(int pos, std::unordered_set<int> &target) {
			if (occupied.find(pos) == occupied.end()) {
				occupied.insert(pos);
				target.insert(pos);
			}
		}

		std::unordered_set<int> mTrees, mBushes, mRocks, mRiver, occupied;
		std::vector<float> soilRichness;
		std::vector<Vector3> treePositions, bushPositions, rockPositions;
	};
}

#endif