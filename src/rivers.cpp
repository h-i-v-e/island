#include <unordered_set>
#include <unordered_map>
#include <queue>

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

	void traceToSea(int source, const Mesh &mesh, const MeshEdgeMap &edges, std::vector<int> &vertices) {
		std::vector<Step> steps(mesh.vertices.size());
		std::priority_queue<Step*, std::vector<Step*>, CompareSteps> fringe;
		steps[source].offset = source;;
		steps[source].visited = true;
		fringe.push(&steps[source]);
		while (!fringe.empty()) {
			if (mesh.vertices[fringe.top()->offset].z <= 0.0f) {
				break;
			}
			Step *step(fringe.top());
			fringe.pop();
			auto range = edges.vertex(step->offset);
			while (range.first != range.second) {
				Step *next = &steps[*range.first];
				if (!next->visited) {
					next->offset = *range.first;
					next->cost = step->cost + mesh.vertices[step->offset].z;
					next->last = step->offset;
					next->visited = true;
					fringe.push(next);
				}
				++range.first;
			}
		}
		if (!fringe.empty()) {
			for (int i = fringe.top()->offset; i != -1; i = steps[i].last) {
				vertices.push_back(i);
			}
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
		sources.reserve(count);
		for (auto i = pots.begin(); i != pots.end(); ++i) {
			if (i->second) {
				sources.push_back(i->first);
			}
		}
	}

	void mergeRivers(std::vector<Rivers::River> &rivers) {
		int count = 0;
		std::vector<std::pair<int, int>> sortlist;
		sortlist.reserve(rivers.size());
		for (int i = 0; i != rivers.size(); ++i) {
			count += rivers[i].size();
			sortlist.emplace_back(i, rivers[i].size());
		}
		std::sort(sortlist.begin(), sortlist.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
			return a.second > b.second;
		});
		std::unordered_map<int, int> visited;
		visited.reserve(count);
		for (const std::pair<int, int> &i : sortlist) {
			Rivers::River &river = rivers[i.first];
			for (int j = 0; j != river.size(); ++j) {
				auto k = visited.find(river[j].first);
				if (k != visited.end()) {
					river.resize(j + 1);
					Rivers::River &join = rivers[k->second];
					int l = 0;
					while (join[l++].first != river[j].first);
					while (l != join.size()) {
						join[l++].second += river[j].second;
					}
					break;
				}
				visited.emplace(river[j].first, i.first);
			}
		}
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
}

Rivers::Rivers(const Mesh &mesh, const MeshEdgeMap &edges) {
	std::vector<int> sources, flow, river;
	findSources(2.5f, sources, flow, mesh, edges);
	mRivers.resize(sources.size());
	int offset = 0;
	for (int source : sources) {
		river.clear();
		traceToSea(source, mesh, edges, river);
		River &r = mRivers[offset++];
		r.reserve(river.size());
		for (int i : river) {
			r.emplace_back(i, flow[i]);
		}
		std::reverse(r.begin(), r.end());
	}
	mergeRivers(mRivers);
}

Rivers::Rivers(const Rivers &rivers, const Mesh &old, const Mesh &nw, const MeshEdgeMap &edges) {
	std::unordered_map<Vector2, int, Hasher<Vector2>> newMap;
	newMap.reserve(nw.vertices.size());
	for (int i = 0; i != nw.vertices.size(); ++i) {
		newMap.emplace(nw.vertices[i].asVector2(), i);
	}
	mRivers.resize(rivers.rivers().size());
	for (int i = 0; i != rivers.rivers().size(); ++i){
		const River &oldRiver = rivers.rivers()[i];
		River &river = mRivers[i];
		river.reserve((oldRiver.size() << 1) - 1);
		int a = newMap[old.vertices[oldRiver[0].first].asVector2()];
		river.emplace_back(a, oldRiver[0].second);
		for (int j = 1; j < oldRiver.size(); ++j) {
			int b = a;
			int flow = oldRiver[j].second;
			a = newMap[old.vertices[oldRiver[j].first].asVector2()];
			river.emplace_back(findJoinVertex(edges, nw, b, a), flow);
			river.emplace_back(a, flow);
		}
	}
}

void Rivers::carveInto(Mesh &mesh, const MeshEdgeMap &em, float depthMultiplier, float maxGradient) const{
	int maxFlow = 0;
	std::vector<std::pair<int, float>> adjustments;
	int count = 0;
	for (const River &river : mRivers) {
		if (river.back().second > maxFlow) {
			maxFlow = river.back().second;
		}
		count += river.size();
	}
	if (maxFlow == 0) {
		return;
	}
	adjustments.reserve(count);
	depthMultiplier /= sqrtf(maxFlow);
	std::vector<int> shortToLongest;
	shortToLongest.reserve(mRivers.size());
	for (int i = 0; i != mRivers.size(); ++i) {
		shortToLongest.push_back(i);
	}
	std::sort(shortToLongest.begin(), shortToLongest.end(), [this](int a, int b) {
		return mRivers[a].size() < mRivers[b].size();
	});
	for (int ix : shortToLongest) {
		const River &river = mRivers[ix];
		for (int i = 0; i < river.size(); ++i) {
			float depth = sqrtf(river[i].second) * depthMultiplier;
			adjustments.emplace_back(river[i].first, std::max(-depth, findLowestNeighbourZ(river[i].first, mesh, em) - depth));
		}
	}
	for (auto i = adjustments.begin(); i != adjustments.end(); ++i) {
		mesh.vertices[i->first].z = i->second;
	}
	adjustments.clear();
	std::reverse(shortToLongest.begin(), shortToLongest.end());
	for (int ix : shortToLongest) {
		const River &river = mRivers[ix];
		for (int i = river.size() - 1; i > 0; --i) {
			//adjustments.emplace_back(river[i].first, findLowestNeighbourZ(river[i].first, mesh, em) - (sqrtf(river[i].second) * depthMultiplier));
			const Vector3 &last = mesh.vertices[river[i - 1].first];
			if (last.z < 0.0f) {
				continue;
			}
			Vector3 shift = last - mesh.vertices[river[i].first];
			float length = shift.asVector2().magnitude();
			float slope = shift.z / length;
			if (slope > maxGradient) {
				mesh.vertices[river[i].first].z -= shift.z - (length * maxGradient);
			}
		}
	}
}

void Rivers::smooth(Mesh &mesh, const MeshEdgeMap &edgeMap) const{
	std::vector<std::pair<int, Vector3>> altered;
	for (const River &river : mRivers) {
		if (river.size() < 3) {
			continue;
		}
		int current = river[0].first, next = river[1].first, last;
		for (int i = 2; i < river.size(); ++i) {
			last = current;
			current = next;
			next = river[i].first;
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
					altered.emplace_back(*neighbours.first, total / count);
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