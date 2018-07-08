#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <stack>

#include "rivers.h"
#include "mesh.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"

#ifndef RIVER_DEPTH_MUL
#define RIVER_DEPTH_MUL 0.5f
#endif

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

	int distanceToSea(int source, const Mesh &mesh) {
		std::vector<Step> steps(mesh.vertices.size());
		std::priority_queue<Step*, std::vector<Step*>, CompareSteps> fringe;
		steps[source].offset = source;
		steps[source].visited = true;
		fringe.push(&steps[source]);
		const MeshEdgeMap &edges = mesh.edgeMap();
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

	void bruteForceToSea(int source, const Mesh &mesh, std::vector<int> &vertices, std::unordered_set<int> &rivers) {
		std::vector<Step> steps(mesh.vertices.size());
		for (int i : vertices) {
			steps[i].visited = true;
		}
		std::priority_queue<Step*, std::vector<Step*>, CompareSteps> fringe;
		steps[source].offset = source;
		steps[source].visited = true;
		fringe.push(&steps[source]);
		std::vector<int> opts;
		const MeshEdgeMap &edges = mesh.edgeMap();
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

	void traceToSea(int source, Mesh &mesh, std::vector<int> &vertices, std::unordered_set<int> &rivers){
		std::vector<int> opt;
		const MeshEdgeMap &edges = mesh.edgeMap();
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
				bruteForceToSea(source, mesh, vertices, rivers);
				return;
			}
			source = next;
		}
	}

	void mapDown(std::vector<int> &down, const Mesh &mesh) {
		down.reserve(mesh.vertices.size());
		const MeshEdgeMap &edges = mesh.edgeMap();
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			auto neighbours = edges.vertex(i);
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

	void calculateFlow(std::vector<int> &down, std::vector<int> &flow, const Mesh &mesh) {
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

	void findSources(float thresholdSd, std::vector<int> &sources, std::vector<int> &flow, const Mesh &mesh) {
		std::vector<int> down;
		mapDown(down, mesh);
		calculateFlow(down, flow, mesh);
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
				sortList.emplace_back(i->first, distanceToSea(i->first, mesh));
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
					verts.emplace(river.vertices[j].index, i);
				}
			}
			for (size_t i = 0; i != rivers.size(); ++i) {
				River &river = *rivers[i];
				auto j = verts.find(river.vertices.back().index);
				if (j != verts.end()) {
					River &downStream = *rivers[j->second];
					int join = downStream.vertices.front().index;
					if (join == j->first) {
						verts[join] = i;
						for (int k = 1; k < downStream.vertices.size(); ++k) {
							verts[downStream.vertices[k].index] = i;
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

	int findJoinVertex(const Mesh &mesh, int a, int b) {
		const MeshEdgeMap &edges = mesh.edgeMap();
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


	float findLowestNeighbourZ(int vert, Mesh &mesh) {
		float lowest = mesh.vertices[vert].z;
		auto neighbours = mesh.edgeMap().vertex(vert);
		while (neighbours.first != neighbours.second) {
			float z = mesh.vertices[*neighbours.first++].z;
			if (z < lowest) {
				z = lowest;
			}
		}
		return lowest;
	}

	void emplaceRiver(int minFlow, std::vector<int> &flow, const std::vector<int> &river, std::vector<std::shared_ptr<River>> &rivers) {
		if (river.size() < 2) {
			return;
		}
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

	void fixInlandSeas(Mesh &mesh) {
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
		const MeshEdgeMap &edges = mesh.edgeMap();
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

	float computeAltMul(float maxHeight, float z, float invMaxHeight) {
		float altMul = (maxHeight - z) * invMaxHeight;
		return altMul * altMul * z * RIVER_DEPTH_MUL;
	}

	void carveRiverValley(std::unordered_set<int> carved, Mesh &mesh, const River &river, float maxHeight, float depthMultiplier) {
		static float maxOut = 4.0f;
		float invMaxHeight = 1.0f / maxHeight;
		const MeshEdgeMap &edges = mesh.edgeMap();
		std::stack<std::pair<int, int>> unfurl;
		for (const auto &i : river.vertices) {
			float carve = sqrtf(i.flow) * depthMultiplier;
			int steps = roundf(carve * maxOut);
			if (steps == 0) {
				continue;
			}
			auto j = edges.vertex(i.index);
			while (j.first != j.second) {
				if (carved.find(*j.first) == carved.end()) {
					unfurl.emplace(*j.first, steps);
					carved.insert(*j.first);
				}
				++j.first;
			}
			while (!unfurl.empty()) {
				auto k = unfurl.top();
				unfurl.pop();
				float z = mesh.vertices[k.first].z;
				float altMul = computeAltMul(maxHeight, mesh.vertices[k.first].z, invMaxHeight);
				mesh.vertices[k.first].z -= altMul * carve * 0.5f;
				if (k.second > 1) {
					auto l = edges.vertex(k.first);
					while (l.first != l.second) {
						if (carved.find(*l.first) == carved.end()) {
							unfurl.emplace(*l.first, k.second - 1);
							carved.insert(*l.first);
						}
						++l.first;
					}
				}
			}
		}
	}

	struct TesselationMapper {
		const Mesh &old, &tesselated;
		std::unordered_map<Vector2, int, Hasher<Vector2>> newMapper;

		TesselationMapper(const Mesh &old, const Mesh &tesselated) : old(old), tesselated(tesselated){
			newMapper.reserve(tesselated.vertices.size());
			for (int i = 0, j = tesselated.vertices.size(); i != j; ++i) {
				newMapper.emplace(tesselated.vertices[i].asVector2(), i);
			}
		}

		int findJoin(int vertA, int vertB) {
			int best = -1;
			float lowest = std::numeric_limits<float>::max();
			const MeshEdgeMap &edgeMap = tesselated.edgeMap();
			auto i = edgeMap.vertex(vertA);
			while (i.first != i.second) {
				auto j = edgeMap.vertex(*i.first);
				while (j.first != j.second) {
					if (*j.first == vertB) {
						float z = tesselated.vertices[*i.first].z;
						if (z < lowest) {
							lowest = z;
							best = *i.first;
						}
						break;
					}
					++j.first;
				}
				++i.first;
			}
			return best;
		}

		void mapRiver(River &river) {
			river.vertices.reserve(river.vertices.size() << 1);
			std::vector<River::Vertex> temp(river.vertices);
			river.vertices.clear();
			River::Vertex current(newMapper[old.vertices[temp.front().index].asVector2()], temp.front().flow);
			river.vertices.push_back(current);
			for (size_t i = 1; i != temp.size(); ++i) {
				current.index = newMapper[old.vertices[temp[i].index].asVector2()];
				current.flow = temp[i].flow;
				int join = findJoin(river.vertices.back().index, current.index);
				river.vertices.emplace_back(join, river.vertices.back().flow);
				river.vertices.push_back(current);
			}
		}
	};
}

Rivers::Rivers(Mesh &mesh, float maxHeight, float flowSDthreshold) {
	fixInlandSeas(mesh);
	std::vector<int> sources, flow, river;
	std::unordered_set<int> bottoms, in;
	findSources(flowSDthreshold, sources, flow, mesh);
	int offset = 0;
	for (int source : sources){
		river.clear();
		traceToSea(source, mesh, river, in);
		emplaceRiver(0, flow, river, mRivers);
	}
	std::sort(mRivers.begin(), mRivers.end(), [&mesh](const River::Ptr &a, const River::Ptr &b) {
		return mesh.vertices[a->vertices.back().index].z > mesh.vertices[b->vertices.back().index].z;
	});
	mergeRivers(mRivers);
	int maxFlow = 0;
	for (const auto &river : mRivers) {
		if (river->vertices.empty()) {
			continue;
		}
		if (river->vertices.back().flow > maxFlow) {
			maxFlow = river->vertices.back().flow;
		}
	}
	depthMultiplier = 1.0f / sqrtf(static_cast<float>(maxFlow));
	this->maxHeight = maxHeight;
}

void Rivers::addaptToTesselation(const Mesh &old, const Mesh &tesselated) {
	TesselationMapper mapper(old, tesselated);
	for (River::Ptr &i : mRivers) {
		mapper.mapRiver(*i);
	}
}

void Rivers::carveInto(Mesh &mesh, float maxGradient) const{
	float invMaxHeight = 1.0f / maxHeight;
	Rivers::RiverList copy(mRivers.begin(), mRivers.end());
	std::sort(copy.begin(), copy.end(), [&mesh](const auto &a, const auto &b) {
		return mesh.vertices[a->vertices.back().index].z < mesh.vertices[b->vertices.back().index].z;
	});
	std::vector<float> surfaces(mesh.vertices.size(), -1.0f);
	/*for (const auto &river : copy) {
		int i = river->vertices.size() - 1;
		Vector3 *last = &mesh.vertices[i];
		for (--i; i >= 0; --i) {
			const River::Vertex &rvert = river->vertices[i];
			Vector3 *current = &mesh.vertices[rvert.index];
			Vector3 direction = *current - *last;
			float mag = direction.magnitude();
			float slope = direction.z / mag;
			if (last->z >= 0.0f && slope > maxGradient) {
				//if (slope < 0.8f) {
					current->z = last->z + mag * maxGradient;
				//}
				//otherwise a waterfall.
			}
			last = current;
		}
	}*/
	for (const auto &river : copy) {
		float surface = -1.0f;
		for (auto i = river->vertices.rbegin(); i != river->vertices.rend(); ++i) {
			float z = mesh.vertices[i->index].z;
			if (z > surface) {
				surface = z;
			}
			surfaces[i->index] = surface;
		}
	}
	for (const auto &river : copy){
		for (auto i = river->vertices.begin(); i != river->vertices.end(); ++i) {
			float z = mesh.vertices[i->index].z, surface = surfaces[i->index];
			float altMul = computeAltMul(maxHeight, mesh.vertices[i->index].z, invMaxHeight);
			float target = altMul * sqrtf(static_cast<float>(i->flow)) * depthMultiplier;
			float depth = surface - z;
			if (depth < target) {
				mesh.vertices[i->index].z -= (target - depth);
			}
			if (surface < z) {
				mesh.vertices[i->index].z = surface;
			}
		}
	}
	for (const auto &river : copy) {
		for (auto i = river->vertices.begin(); i != river->vertices.end(); ++i) {
			float altMul = computeAltMul(maxHeight, mesh.vertices[i->index].z, invMaxHeight);
			i->surface = surfaces[i->index] - altMul * sqrtf(static_cast<float>(i->flow)) * depthMultiplier * 0.5f;
		}
	}
	std::unordered_set<int> visited;
	for (const auto &i : mRivers) {
		for (const auto &j : i->vertices) {
			visited.insert(j.index);
		}
	}
	for (const auto &i : mRivers) {
		carveRiverValley(visited, mesh, *i, maxHeight, depthMultiplier);
	}
}

void Rivers::smooth(Mesh &mesh) const{
	std::vector<std::pair<int, Vector3>> altered;
	const MeshEdgeMap &edgeMap = mesh.edgeMap();
	for (const auto &river : mRivers) {
		if (river->vertices.size() < 3) {
			continue;
		}
		int current = river->vertices[0].index, next = river->vertices[1].index, last;
		for (int i = 2; i < river->vertices.size(); ++i) {
			last = current;
			current = next;
			next = river->vertices[i].index;
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

Mesh &Rivers::getMesh(const River &river, const Mesh &mesh, Mesh &out) const{
	int num = 0;
	const MeshTriangleMap &triangleMap = mesh.triangleMap();
	for (auto i = river.vertices.begin(); i != river.vertices.end(); ++i) {
		auto ends = triangleMap.vertex(i->index);
		num += ends.second - ends.first;
	}
	std::unordered_map<int, int> vertexMap;
	vertexMap.reserve(num * 3);
	out.triangles.reserve(num * 3);
	std::unordered_set<int> added;
	added.reserve(num);
	out.vertices.reserve(num + 2);
	for (auto i = river.vertices.rbegin(); i != river.vertices.rend(); ++i) {
		float z = i->surface;
		auto ends = triangleMap.vertex(i->index);
		while (ends.first != ends.second) {
			int triOff = *ends.first++;
			if (added.find(triOff) == added.end()) {
				added.insert(triOff);
				for (int i = 0; i != 3; ++i) {
					int vert = mesh.triangles[triOff + i];
					auto j = vertexMap.find(vert);
					if (j == vertexMap.end()) {
						int val = out.vertices.size();
						out.triangles.push_back(val);
						vertexMap.emplace(vert, val);
						Vector3 v3 = mesh.vertices[vert];
						v3.z = z;
						out.vertices.push_back(v3);
					}
					else {
						out.triangles.push_back(j->second);
					}
				}
			}
		}
	}
	out.calculateNormals();
	return out;
}