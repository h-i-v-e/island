#ifndef TRIANGULATED_TERRAIN_GRAPH
#define TRIANGULATED_TERRAIN_GRAPH

#include "dalauney_triangulation.h"
#include "grid.h"
#include "river.h"

namespace motu {

	struct Mesh;
	class TerrainGraph;
	class Raster;

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

		Rivers::Edges &findRivers(Rivers::Edges &edges, float thresholdStandardDeviations = 3.0f);

	private:
		Triangulation *mTriangulation;
	};
}
#endif