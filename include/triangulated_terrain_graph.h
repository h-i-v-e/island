#ifndef TRIANGULATED_TERRAIN_GRAPH
#define TRIANGULATED_TERRAIN_GRAPH

#include "dalauney_triangulation.h"
#include "grid.h"
#include "river.h"
#include "triangle3.h"
#include <vector>

namespace motu {

	struct Mesh;
	class TerrainGraph;

	class TriangulatedTerrainGraph {
	public:

		struct VertexData;

		typedef DalauneyTriangulation<EmptyData, VertexData> Triangulation;

		struct VertexData {
			Triangulation::Vertex *down;
			float z;
			uint32_t flow;

			VertexData() : down(nullptr), z(0.0f), flow(0) {}
		};

		TriangulatedTerrainGraph(const TerrainGraph &graph);

		TriangulatedTerrainGraph(const TriangulatedTerrainGraph &graph);

		~TriangulatedTerrainGraph() {
			delete mTriangulation;
		}

		static Vector3 vector3(const Triangulation::Vertex &vert) {
			return Vector3(vert.position().x, vert.position().y, vert.data().z);
		}

		const Triangulation &triangulation() const {
			return *mTriangulation;
		}

		Triangulation &triangulation() {
			return *mTriangulation;
		}

		void smooth();

		void erode(int drops, float carryCapacity);

		void toMesh(Mesh &mesh) const;

		void copyBackZValues(const Grid<Vector3> &);

		typedef std::pair<Vector2, Vector2> RiverSection;
		typedef std::vector<RiverSection> RiverSections;

		struct RiverBed {
			Triangulation::Vertex *last;
			float lowestZ;

			RiverBed(Triangulation::Vertex *last) : last(last), lowestZ(last->data().z) {}
		};

		struct LastList : public std::vector<std::pair<Triangulation::Vertex *, RiverBed>> {
			float flowMultiplier;

			RiverSections &copyToRiverSections(RiverSections &riverSections) const{
				riverSections.reserve(size());
				for (auto i : *this) {
					riverSections.emplace_back(i.second.last->position(), i.first->position());
				}
				return riverSections;
			}
		};

		void fillLastList(LastList &, float thresholdStandardDeviations = 3.0f);

		void fillLastListInterpolated(RiverSections &, LastList &);

		void carveRiverBeds(LastList &);

		void smoothRiverBeds(LastList &);

		Mesh &generateRiverMesh(LastList &lastList, Mesh &mesh);

	private:
		Triangulation *mTriangulation;
	};
}
#endif