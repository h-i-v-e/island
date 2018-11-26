#ifndef MOTU_MESH_UTILS
#define MOTU_MESH_UTILS

#include <stack>

#include "Mesh.h"

namespace motu {

	Mesh &smoothPeeks(Mesh &mesh, const MeshEdgeMap &);

	struct DefaultDistInc {
		float operator()(float old) const {
			return old + 1.0f;
		}
	};

	template <class In, class DistInc = DefaultDistInc>
	float computeDistance(In in, const Mesh &mesh, const MeshEdgeMap &edges, std::vector<float> &distanceVec, DistInc distInc = DistInc(), float distance = 0) {

		typedef std::stack<int> Stack;

		Stack circles[2];
		Stack *last = circles;
		Stack *next = circles + 1;

		for (int i = 0; i != mesh.vertices.size(); ++i) {
			auto neighbours = edges.vertex(i);
			while (neighbours.first != neighbours.second) {
				if (in(*neighbours.first++)) {
					distanceVec[i] = 0.0f;
					last->push(i);
					break;
				}
			}
		}
		do {
			distance = distInc(distance);
			while (!last->empty()) {
				int i = last->top();
				last->pop();
				auto neighbours = edges.vertex(i);
				while (neighbours.first != neighbours.second) {
					if (distance < distanceVec[*neighbours.first]) {
						distanceVec[*neighbours.first] = distance;
						next->push(*neighbours.first);
					}
					++neighbours.first;
				}
			}
			Stack *swap = last;
			last = next;
			next = swap;
		} while (!last->empty());
		return distance;
	}
}

#endif // !MOTU_MESH_UTILS
