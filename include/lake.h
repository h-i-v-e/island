#ifndef MOTU_LAKE_H
#define MOTU_LAKE_H

#include <memory>
#include <vector>
#include <unordered_set>
#include <algorithm>

namespace motu {
	struct Mesh;
	class MeshEdgeMap;
	struct MeshTriangleMap;

	struct Lake {
		typedef std::shared_ptr<Lake> Ptr;
		typedef std::vector<Ptr> Lakes;

		Lake(int flow, int outflow, const std::unordered_set<int> &lakeBedVertices) : flow(flow), outflow(outflow){
			this->lakeBedVertices.reserve(lakeBedVertices.size());
			std::copy(lakeBedVertices.begin(), lakeBedVertices.end(), std::back_inserter(this->lakeBedVertices));
		}

		void carveInto(Mesh &mesh);

		static void carveInto(Mesh &mesh, const Lakes &lakes) {
			for (auto i = lakes.begin(); i != lakes.end(); (*i++)->carveInto(mesh));
		}

		static void mergeLakes(const Mesh &mesh, Lakes &lakes);

		std::unordered_set<int> &fillLakeVertexSet(const Lake::Lakes &lakes, std::unordered_set<int> &out);

		int outflow, flow;
		std::vector<int> lakeBedVertices;

		Mesh &createMesh(const Mesh &in, const MeshTriangleMap &, Mesh &mesh) const;
	};
}

#endif