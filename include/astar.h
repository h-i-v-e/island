#ifndef ASTAR_H
#define ASTAR_H

#include <queue>
#include <unordered_set>

#include "util.h"

namespace motu {
	template <class State, class Branch>
	class AStar {
	public:
		void solve(State &initialState, const State &goal) {
			std::priority_queue<State> stateQueue;
			std::unordered_set<State, Hasher<State>> visited;
			stateQueue.push(intitialState);
			visited.insert(initialState);
			while (!stateQueue.empty()) {
				State current(stateQueue.top());
				stateQueue.pop();
				if (current == goal) {
					return;
				}
				for (const State &next : branch(current)) {
					if (visited.find(next) == visited.end()) {
						stateQueue.push(next);
						visited.insert(next);
					}
				}
			}
		}
	private:
		Branch branch;
	};
}

#endif