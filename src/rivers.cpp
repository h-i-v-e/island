#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>

#include "rivers.h"
#include "mesh.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"

using namespace motu;

namespace {
	struct Step {
		int offset, last;
		float cost;
		bool visited;

		Step() : offset(-1), last(-1), cost(0.0f), visited(false) {}

		bool operator<(const Step &step) const {
			return cost < step.cost;
		}
	};

	struct CompareSteps {
		bool operator()(const Step *a, const Step *b) const {
			return a->cost > b->cost;
		}
	};

	int distanceToSea(int source, const Mesh &mesh, const MeshEdgeMap &edges) {
		std::vector<Step> steps(mesh.vertices.size());
		std::priority_queue<Step*, std::vector<Step*>, CompareSteps> fringe;
		steps[source].offset = source;
		steps[source].visited = true;
		fringe.push(&steps[source]);
		while (!fringe.empty()) {
			Step *step(fringe.top());
			fringe.pop();
			auto range = edges.vertex(step->offset);
			while (range.first != range.second) {
				Step *next = &steps[*range.first];
				if (!next->visited) {
					next->offset = *range.first;
					next->cost = step->cost + mesh.vertices[next->offset].z;
					next->last = step->offset;
					next->visited = true;
					fringe.push(next);
				}
				++range.first;
			}
		}
		if (!fringe.empty()) {
			int count = 0;
			for (int i = fringe.top()->offset; i != -1; i = steps[i].last) {
				++count;
			}
			return count;
		}
	}

	void bruteForceToSea(int source, const Mesh &mesh, const MeshEdgeMap &edges, std::vector<int> &vertices, std::unordered_set<int> &rivers) {
		std::vector<Step> steps(mesh.vertices.size());
		for (int i : vertices) {
			steps[i].visited = true;
		}
		std::priority_queue<Step*, std::vector<Step*>, CompareSteps> fringe;
		steps[source].offset = source;
		steps[source].visited = true;
		fringe.push(&steps[source]);
		std::vector<int> opts;
		while (!fringe.empty()) {
			auto i = rivers.find(fringe.top()->offset);
			if (mesh.vertices[fringe.top()->offset].z <= 0.0f || i != rivers.end()) {
				break;
			}
			Step *step(fringe.top());
			fringe.pop();
			auto range = edges.vertex(step->offset);
			while (range.first != range.second) {
				Step *next = &steps[*range.first];
				if (!next->visited) {
					next->offset = *range.first;
					if (rivers.find(next->offset) != rivers.end()) {
						next->last = step->offset;
						opts.push_back(next->offset);
					}
					else {
						next->cost = step->cost + mesh.vertices[next->offset].z;
						next->last = step->offset;
						next->visited = true;
						fringe.push(next);
					}
				}
				++range.first;
			}
			if (!opts.empty()) {
				float lowest = std::numeric_limits<float>::max();
				for (int i : opts) {
					float z = mesh.vertices[i].z;
					if (z < lowest) {
						lowest = z;
						source = i;
					}
				}
				std::vector<int> tmp;
				for (int i = source; i != -1; i = steps[i].last) {
					rivers.insert(i);
					tmp.push_back(i);
				}
				std::copy(tmp.rbegin(), tmp.rend(), std::back_inserter(vertices));
				return;
				//vertices.push_back(source);
			}
		}
		if (!fringe.empty()) {
			std::vector<int> tmp;
			for (int i = fringe.top()->offset; i != -1; i = steps[i].last) {
				rivers.insert(i);
				tmp.push_back(i);
			}
			std::copy(tmp.rbegin(), tmp.rend(), std::back_inserter(vertices));
		}
	}

	/*struct OutFlowStep {
		int vert;
		float z;

		OutFlowStep(int vert, float z) : vert(vert), z(z) {}

		constexpr bool operator < (const OutFlowStep &out) const {
			return z < out.z;
		}
	};*/

	void traceToSea(int source, Mesh &mesh, const MeshEdgeMap &edges, std::vector<int> &vertices, std::unordered_set<int> &rivers, Lake::Lakes &lakes){
		std::vector<int> opt;
		for (float z = mesh.vertices[source].z; z >= 0.0f; z = mesh.vertices[source].z) {
			vertices.push_back(source);
			auto n = edges.vertex(source);
			int next = -1;
			while (n.first != n.second) {
				int vert = *n.first++;
				auto i = rivers.find(vert);
				if (i != rivers.end()) {
					opt.push_back(*i);
				}
				float nz = mesh.vertices[vert].z;
				if (nz < z) {
					z = nz;
					next = vert;
				}
			}
			if (!opt.empty()) {
				float lowest = mesh.vertices[source].z;
				for (int o : opt) {
					if (mesh.vertices[o].z <= lowest) {
						lowest = z;
						next = o;
					}
				}
				if (next != -1) {
					vertices.push_back(next);
					for (int i : vertices) {
						rivers.insert(i);
					}
					return;
				}
			}
			if (next == -1) {
				vertices.pop_back();
				bruteForceToSea(source, mesh, edges, vertices, rivers);
				return;
			}
			source = next;
		}
	}

	void mapDown(std::vector<int> &down, const Mesh &mesh, const MeshEdgeMap &edges) {
		down.reserve(mesh.vertices.size());
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			std::pair<const int*, const int*> neighbours(edges.vertex(i));
			int d = -1;
			float z = mesh.vertices[i].z;
			while (neighbours.first != neighbours.second) {
				int offset = *neighbours.first;
				float oz = mesh.vertices[offset].z;
				if (oz < z) {
					z = oz;
					d = offset;
				}
				++neighbours.first;
			}
			down.push_back(d);
		}
	}

	void calculateFlow(std::vector<int> &down, std::vector<int> &flow, const Mesh &mesh, const MeshEdgeMap &edges) {
		flow.resize(down.size(), 0);
		for (int i = 0; i != down.size(); ++i) {
			for (int next = down[i]; next != -1; next = down[next]) {
				++flow[next];
			}
		}
	}

	inline float computeFlowSDAndMean(std::vector<int> &flow, float &mean) {
		int total = 0;
		for (int i : flow) {
			total += i;
		}
		mean = static_cast<float>(total) / flow.size();
		float sumVariance = 0.0f;
		for (int i : flow) {
			float v = i - mean;
			sumVariance += v * v;
		}
		return sqrtf(sumVariance / total);
	}

	void findSources(float thresholdSd, std::vector<int> &sources, std::vector<int> &flow, const Mesh &mesh, const MeshEdgeMap &edges) {
		std::vector<int> down;
		mapDown(down, mesh, edges);
		calculateFlow(down, flow, mesh, edges);
		float mean;
		thresholdSd *= computeFlowSDAndMean(flow, mean);
		std::unordered_map<int, bool> pots;
		pots.reserve(down.size());
		int count = 0;
		for (int i = 0; i != flow.size(); ++i) {
			if (flow[i] - mean >= thresholdSd && mesh.vertices[i].z > 0.0f) {
				pots.emplace(i, true);
				++count;
			}
		}
		for (int i = 0; i != flow.size(); ++i) {
			if (flow[i] - mean >= thresholdSd && pots[i] && mesh.vertices[i].z > 0.0f) {
				for (int next = down[i]; next != -1; next = down[next]) {
					auto j = pots.find(next);
					if (j != pots.end() && j->second) {
						j->second = false;
						--count;
					}
				}
			}
		}
		std::vector<std::pair<int, int>> sortList;
		sortList.reserve(count);
		for (auto i = pots.begin(); i != pots.end(); ++i) {
			if (i->second) {
				sortList.emplace_back(i->first, distanceToSea(i->first, mesh, edges));
			}
		}
		std::sort(sortList.begin(), sortList.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
			return a.second < b.second;
		});
		sources.reserve(sortList.size());
		for (auto i : sortList) {
			sources.push_back(i.first);
		}
	}

	void mergeRivers(Rivers::RiverList &rivers) {
		std::unordered_map<int, int> verts;
		std::unordered_set<int> removed;
		do {
			verts.clear();
			removed.clear();
			for (size_t i = 0; i != rivers.size(); ++i) {
				const River &river = *rivers[i];
				for (int j = 0; j != river.vertices.size() - 1; ++j) {
					verts.emplace(river.vertices[j].first, i);
				}
			}
			for (size_t i = 0; i != rivers.size(); ++i) {
				River &river = *rivers[i];
				auto j = verts.find(river.vertices.back().first);
				if (j != verts.end()) {
					River &downStream = *rivers[j->second];
					int join = downStream.vertices.front().first;
					if (join == j->first) {
						verts[join] = i;
						for (int k = 1; k < downStream.vertices.size(); ++k) {
							verts[downStream.vertices[k].first] = i;
							river.vertices.push_back(downStream.vertices[k]);
						}
						removed.insert(j->second);
						river.join = -1;
					}
					else {
						river.join = j->second;
					}
				}
			}
			Rivers::RiverList swap(rivers);
			rivers.clear();
			for (size_t i = 0; i != swap.size(); ++i) {
				if (removed.find(static_cast<int>(i)) == removed.end()) {
					rivers.push_back(swap[i]);
				}
			}
		}
		while (!removed.empty());
	}

	int findJoinVertex(const MeshEdgeMap &edges, const Mesh &mesh, int a, int b) {
		auto aends = edges.vertex(a), bends = edges.vertex(b);
		int best = -1;
		float minZ = std::numeric_limits<float>::max();
		while (aends.first != aends.second) {
			for (auto i = bends.first; i != bends.second; ++i) {
				if (*aends.first == *i && mesh.vertices[*i].z < minZ) {
					minZ = mesh.vertices[*i].z;
					best = *i;
				}
			}
			++aends.first;
		}
		return best;
	}


	float findLowestNeighbourZ(int vert, Mesh &mesh, const MeshEdgeMap &em) {
		float lowest = mesh.vertices[vert].z;
		auto neighbours = em.vertex(vert);
		while (neighbours.first != neighbours.second) {
			float z = mesh.vertices[*neighbours.first++].z;
			if (z < lowest) {
				z = lowest;
			}
		}
		return lowest;
	}

	void emplaceRiver(int minFlow, std::vector<int> &flow, const std::vector<int> &river, std::vector<std::shared_ptr<River>> &rivers) {
		rivers.emplace_back(std::make_shared<River>());
		River &r = *rivers.back();
		r.vertices.reserve(river.size());
		for (int i : river) {
			int f = flow[i];
			if (f < minFlow) {
				f = flow[i] = minFlow;
			}
			else {
				minFlow = f;
			}
			r.vertices.emplace_back(i, f);
		}
	}

	void fixInlandSeas(Mesh &mesh, const MeshEdgeMap &edges) {
		Mesh::PerimeterSet perimeter;
		mesh.getPerimeterSet(perimeter);
		std::vector<bool> visited(mesh.vertices.size(), false);
		std::stack<int> waiting;
		for (int vert : perimeter) {
			if (mesh.vertices[vert].z < 0.0f) {
				waiting.push(vert);
				visited[vert] = true;
			}
		}
		while (!waiting.empty()) {
			int top = waiting.top();
			waiting.pop();
			perimeter.insert(top);
			auto neighbours = edges.vertex(top);
			while (neighbours.first != neighbours.second) {
				int vert = *neighbours.first++;
				if (visited[vert]) {
					continue;
				}
				if (mesh.vertices[vert].z < 0.0f) {
					waiting.push(vert);
					visited[vert] = true;
				}
			}
		}
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			if (visited[i]) {
				continue;
			}
			if (mesh.vertices[i].z < FLT_EPSILON) {
				mesh.vertices[i].z = FLT_EPSILON;
			}
		}
	}
}

Rivers::Rivers(Mesh &mesh, const MeshEdgeMap &edges, Lake::Lakes &lakes, float flowSDthreshold) {
	fixInlandSeas(mesh, edges);
	std::vector<int> sources, flow, river;
	std::unordered_set<int> bottoms, in;
	findSources(flowSDthreshold, sources, flow, mesh, edges);
	int offset = 0;
	for (int source : sources){
		river.clear();
		traceToSea(source, mesh, edges, river, in, lakes);
		emplaceRiver(0, flow, river, mRivers);
	}
	mergeRivers(mRivers);
}

void Rivers::carveInto(Mesh &mesh, const MeshEdgeMap &em, float depthMultiplier, float maxGradient) const{
	int maxFlow = 0;
	for (const auto &river : mRivers) {
		if (river->vertices.empty()) {
			continue;
		}
		if (river->vertices.back().second > maxFlow) {
			maxFlow = river->vertices.back().second;
		}
	}
	if (maxFlow == 0) {
		return;
	}
	depthMultiplier /= sqrtf(static_cast<float>(maxFlow));
	std::vector<float> surfaces(mesh.vertices.size(), -1.0f);
	for (const auto &river : mRivers) {
		float surface = -1.0f;
		for (auto i = river->vertices.rbegin(); i != river->vertices.rend(); ++i) {
			float z = mesh.vertices[i->first].z;
			if (z > surface) {
				surface = z;
			}
			surfaces[i->first] = surface;
		}
	}
	for (const auto &river : mRivers) {
		float surface = -1.0f;
		for (auto i = river->vertices.begin(); i != river->vertices.end(); ++i) {
			float z = mesh.vertices[i->first].z, surface = surfaces[i->first], target = sqrtf(static_cast<float>(i->second)) * depthMultiplier;
			float depth = surface - z;
			if (depth < target) {
				mesh.vertices[i->first].z -= (target - depth);
			}
		}
	}
}

void Rivers::smooth(Mesh &mesh, const MeshEdgeMap &edgeMap) const{
	std::vector<std::pair<int, Vector3>> altered;
	for (const auto &river : mRivers) {
		if (river->vertices.size() < 3) {
			continue;
		}
		int current = river->vertices[0].first, next = river->vertices[1].first, last;
		for (int i = 2; i < river->vertices.size(); ++i) {
			last = current;
			current = next;
			next = river->vertices[i].first;
			const Vector3 &a = mesh.vertices[last], &b = mesh.vertices[current], &c = mesh.vertices[next];
			altered.emplace_back(current, (a + b + c) / 3.0f);
			if (c.z < 0.0f) {
				continue;
			}
			auto neighbours = edgeMap.vertex(current);
			while (neighbours.first != neighbours.second) {
				if (*neighbours.first != last && *neighbours.first != next) {
					auto nn = edgeMap.vertex(*neighbours.first);
					Vector3 total(mesh.vertices[*neighbours.first]);
					int count = 1;
					while (nn.first != nn.second) {
						total += mesh.vertices[*(nn.first++)];
						++count;
					}
					altered.emplace_back(*neighbours.first, total / static_cast<float>(count));
				}
				++neighbours.first;
			}
		}
	}
	for (const std::pair<int, Vector3> &i : altered) {
		mesh.vertices[i.first] = i.second;
	}
}

Mesh &Rivers::getMesh(const River &river, Mesh &mesh, const MeshTriangleMap &triangleMap) {
	/*int num = 0;
	for (auto i = river.begin(); i != river.end(); ++i) {
		auto ends = triangleMap.vertex(i->first);
		num += ends.second - ends.first;
	}
	std::unordered_set<int> triangles;
	triangles.reserve(num);
	for (auto i = river.begin(); i != river.end(); ++i) {
		auto ends = edgeMap.vertex(i->first);
		num += ends.second - ends.first;
	}*/
	return mesh;
}