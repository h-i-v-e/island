#ifndef MOTU_ASTAR
#define MOTU_ASTAR

#include <unordered_set>
#include <queue>
#include <vector>

namespace motu {
	template <class Spec>
	class AStar {
	public:
		typedef std::vector<typename Spec::NodeType> Path;
	private:
		Spec & spec;

		struct Step {
			int last;
			typename Spec::NodeType node;
			typename Spec::CostType cost;

			bool operator<(const Step &other) const {
				return cost > other.cost;
			}

			Step() {}

			Step(int last, typename Spec::NodeType node, typename Spec::CostType cost) : last(last), node(node), cost(cost) {}
		};

		std::priority_queue<Step> fringe;
		std::vector<Step> steps;

		void copyOutPath(Path &path) {
			const Step *step = &steps.back();
			for (;;) {
				path.push_back(step->node);
				if (step->last == -1) {
					std::reverse(path.begin(), path.end());
					return;
				}
				step = steps.data() + step->last;
			}
		}
		
	public:

		AStar(Spec &spec) : spec(spec) {}

		bool search(typename Spec::NodeType initialState, Path &path) {
			spec.visit(initialState);
			fringe.emplace(-1, initialState, spec.hueristic(initialState));
			for (;;) {
				if (fringe.empty()) {
					return false;
				}
				int last = static_cast<int>(steps.size());
				steps.push_back(fringe.top());
				if (spec.goal(steps.back().node)) {
					copyOutPath(path);
					return true;
				}
				fringe.pop();
				typename Spec::NeighboursType neighbours = spec.neighbours(steps.back().node);
				for (auto i = neighbours.begin(); i != neighbours.end(); ++i) {
					typename Spec::NodeType next = *i;
					if (spec.visited(next)) {
						continue;
					}
					spec.visit(next);
					fringe.emplace(last, next, spec.hueristic(next));
				}
			}
		}
	};
}

#endif