#include <unordered_map>
#include <algorithm>

#include "river_mesh_builder.h"
#include "rivers.h"
#include "mesh.h"
#include "mesh_triangle_map.h"
#include "decoration.h"

using namespace motu;

namespace {
	struct RiverMeshBuilder{
		typedef std::unordered_set<int> ISet;
		typedef std::unordered_map<int, int> IMap;
		typedef std::vector<std::pair<int, int>> SideList;
		typedef std::unordered_map<int, Vector2> UVMap;

		float flowMul, uvScale;
		MeshWithUV *output;
		Mesh &terrain;
		const MeshEdgeMap &mem;
		const MeshTriangleMap &mtm;
		IMap addedVerts, addedTriangles;
		ISet added, waterfalls;
		IMap leftVerts, rightVerts;
		std::vector<float> flows;
		UVMap uvs;
		SideList leftSide, rightSide;

		RiverMeshBuilder(float flowMul, float uvScale, MeshWithUV *ouput, Mesh &terrain, const MeshEdgeMap &mem, const MeshTriangleMap &mtm) :
			flowMul(flowMul), uvScale(uvScale), output(output), terrain(terrain),  mem(mem), mtm(mtm) {
		}

		Vector3 smooth(int vert, const IMap &side) {
			Vector3 output = terrain.vertices[vert];
			int count = 1;
			for (auto i = mem.vertex(vert); i.first != i.second; ++i.first) {
				int offset = *i.first;
				if (side.find(offset) != side.end()) {
					output += terrain.vertices[offset];
					++count;
				}
			}
			return output / static_cast<float>(count);
		}

		void loadSideList(const River::Vertices &river, const IMap &side, SideList &sideList) {
			size_t len = side.size();
			sideList.reserve(side.size());
			sideList.clear();
			for (auto i : side) {
				sideList.push_back(i);
			}
			std::sort(sideList.begin(), sideList.end(), [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
				return a.second < b.second;
			});
			for (bool sorting = true; sorting;){
				sorting = false;
				for (size_t i = 1; i != len; ++i) {
					int pos = sideList[i].second;
					if (sideList[i - 1].second == pos) {
						if (pos == 0) {
							const Vector3 &comp = terrain.vertices[river[pos + 1].index];
							if ((terrain.vertices[sideList[i].first] - comp).squareMagnitude() > (terrain.vertices[sideList[i + 1].first] - comp).squareMagnitude()) {
								sorting = true;
								std::swap(sideList[i], sideList[i + 1]);
							}
						}
						else {
							const Vector3 &comp = terrain.vertices[river[pos - 1].index];
							if ((terrain.vertices[sideList[i].first] - comp).squareMagnitude() < (terrain.vertices[sideList[i - 1].first] - comp).squareMagnitude()) {
								sorting = true;
								std::swap(sideList[i], sideList[i - 1]);
							}
						}
					}
				}
			}
		}

		void smooth(const River &river, const IMap &side) {
			std::vector<std::pair<int, Vector3>> adjusted(side.size());
			size_t offset = 0;
			for (auto i : side) {
				adjusted[offset++] = std::make_pair(i.first, smooth(i.first, side));
			}
			for (const auto &i : adjusted) {
				terrain.vertices[i.first] = i.second;
			}
		}

		Vector3 smooth(int vert, const MeshEdgeMap &mem) {
			Vector3 output = terrain.vertices[vert];
			auto i = mem.vertex(vert);
			size_t count = (i.second - i.first) + 1;
			while (i.first != i.second) {
				output += terrain.vertices[*i.first++];
			}
			return output / static_cast<float>(count);
		}

		Vector3 smooth(int vert) {
			return smooth(vert, mem);
		}

		void smooth(const River &river) {
			size_t len = river.vertices.size();
			std::vector<Vector3> adjusted(len);
			for (size_t i = 0; i != len; ++i) {
				int vert = river.vertices[i].index;
				adjusted[i] = smooth(vert);
			}
			for (size_t i = 0; i != len; ++i) {
				terrain.vertices[river.vertices[i].index] = adjusted[i];
			}
		}

		void assignLeftAndRight(int offset, int current, const Vector3 &last) {
			Vector3 centre = terrain.vertices[current];
			Vector3 dir = last - centre;
			for (auto i = mem.vertex(current); i.first != i.second; ++i.first) {
				int vert = *i.first;
				if (added.find(vert) != added.end()) {
					continue;
				}
				if ((terrain.vertices[vert] - centre).cross(dir).z < 0.0f) {
					leftVerts.emplace(vert, offset);
				}
				else {
					rightVerts.emplace(vert, offset);
				}
			}
		}

		void assignLeftAndRight(const River &river) {
			int last = river.vertices[0].index;
			Vector3 v3 = terrain.vertices[last];
			assignLeftAndRight(0, last, v3 - (terrain.vertices[river.vertices[1].index] - v3));
			for (size_t i = 1, j = river.vertices.size(); i < j; ++i) {
				int next = river.vertices[i].index;
				assignLeftAndRight(static_cast<int>(i), next, terrain.vertices[last]);
				last = next;
			}
		}

		void moveFromTo(int move, const Vector3 &to, float dist) {
			Vector3 &target = terrain.vertices[move];
			target += (to - target) * dist;
		}

		void setWidth(int last, int current, int next, float flow) {
			if (flow <= 0.0f) {
				return;
			}
			const Vector3 &v3 = terrain.vertices[current];
			auto l = mem.vertex(last), n = mem.vertex(next);
			int count = 0;
			auto i = mem.vertex(current);
			int a = -1, b = -1;
			float ad, bd;
			for (auto i = mem.vertex(current); i.first != i.second; ++i.first) {
				int vert = *i.first;
				if (vert == next || vert == last) {
					continue;
				}
				if (!std::find(l.first, l.second, vert)) {
					continue;
				}
				if (!std::find(n.first, n.second, vert)) {
					continue;
				}
				float dist = (v3 - terrain.vertices[vert]).squareMagnitude();
				if (a == -1) {
					a = vert;
					ad = dist;
					continue;
				}
				if (b == -1 || dist < bd) {
					if (dist < ad) {
						b = a;
						bd = ad;
						a = vert;
						ad = dist;
						continue;
					}
					b = vert;
					bd = dist;
				}
			}
			moveFromTo(a, v3, flow);
			moveFromTo(b, v3, flow);
		}

		void setWidths(const River::Vertices &verts, bool skipLast) {
			size_t len = verts.size() - (skipLast ? 2 : 1);
			for (size_t i = 1; i < len; ++i) {
				setWidth(verts[i - 1].index, verts[i].index, verts[i + 1].index, 0.75f - flows[i]);
			}
		}

		float setUVs(const River::Vertices &verts, const IMap &side, float uOff, SideList &sideList) {
			loadSideList(verts, side, sideList);
			auto last = sideList.back();
			auto itr = uvs.find(last.first);
			if (itr == uvs.end()) {
				float xDist = (terrain.vertices[verts[last.second].index] - terrain.vertices[last.first]).magnitude();
				itr = uvs.insert(itr, std::make_pair(last.first, Vector2(uOff * xDist, 0.0f)));
			}
			size_t len = sideList.size();
			if (len < 2) {
				return itr->second.y;
			}
			len -= 2;
			for (int i = static_cast<int>(len); i >= 0; --i) {
				auto current = sideList[i];
				const Vector3 &v3 = terrain.vertices[current.first];
				float distance = (terrain.vertices[last.first] - v3).magnitude() * uvScale;
				float xDist = (terrain.vertices[verts[current.second].index] - v3).magnitude() * uOff;
				itr = uvs.insert(uvs.end(), std::make_pair(current.first, Vector2(xDist, itr->second.y - distance)));
				last = current;
			}
			return itr->second.y;
		}

		const void interpUV(int vert) {
			Vector2 total(0.0f, 0.0f);
			int count = 0;
			for (auto i = mem.vertex(vert); i.first != i.second; ++i.first) {
				auto val = uvs.find(*i.first);
				if (val != uvs.end()) {
					total += val->second;
					++count;
				}
			}
			uvs.emplace(vert, count > 0 ? total / static_cast<float>(count) : total);
		}

		float findClosestPointOnOppositeBank(const River::Vertices &verts, IMap::const_iterator itr, const IMap &side) {
			const Vector3 &pos = terrain.vertices[itr->first];
			float minDist = std::numeric_limits<float>::max(), best = uvs[itr->first].y;
			for (auto i = mem.vertex(verts[itr->second].index); i.first != i.second; ++i.first) {
				int idx = *i.first;
				if (side.find(idx) == side.end() && uvs.find(idx) != uvs.end()) {
					float dist = (terrain.vertices[idx] - pos).squareMagnitude();
					if (dist < minDist) {
						best = uvs[idx].y;
					}
				}
			}
			return best;
		}

		void correctUVs(const River::Vertices &verts, const IMap &side) {
			for (auto i = side.begin(); i != side.end(); ++i) {
				uvs[i->first].y = findClosestPointOnOppositeBank(verts, i, side);
			}
		}

		void deWeird(const SideList &side) {
			auto itr = uvs.find(side.front().first);
			for (size_t i = 1, j = side.size(); i < j; ++i) {
				int idx = side[i].first;
				auto iitr = uvs.find(idx);
				if (itr->second.y >= iitr->second.y) {
					float total = 0.0f;
					int count = 0;
					for (auto j = mem.vertex(idx); j.first != j.second; ++j.first) {
						auto k = uvs.find(*j.first);
						if (k != uvs.end()) {
							total += k->second.y;
							++count;
						}
					}
					if (count) {
						iitr->second.y = total / static_cast<float>(count);
					}
				}
				itr = iitr;
			}
		}

		void setUVs(const River::Vertices &verts) {
			setUVs(verts, leftVerts, uvScale, leftSide);
			setUVs(verts, rightVerts, -uvScale, rightSide);
			if (leftSide.size() < rightSide.size()) {
				correctUVs(verts, leftVerts);
			}
			else {
				correctUVs(verts, rightVerts);
			}
			deWeird(leftSide);
			deWeird(rightSide);
			for (const auto &i : verts) {
				interpUV(i.index);
			}
		}

		void squish(int vert) {
			if (added.find(vert) != added.end()) {
				return;
			}
			added.insert(vert);
			float lowest = terrain.vertices[vert].z;
			auto n = mem.vertex(vert);
			for (auto i = n.first; i != n.second; ++i) {
				float f = terrain.vertices[*i].z;
				if (f < lowest) {
					lowest = f;
				}
			}
			terrain.vertices[vert].z = lowest;
			for (auto i = n.first; i != n.second; ++i) {
				terrain.vertices[*i].z = lowest;
			}
		}

		float computeDepthMul(const River &river) const{
			size_t len = river.vertices.size() - 1;
			if (len < 1) {
				return 0.0f;
			}
			Vector3 v3 = terrain.vertices[river.vertices[0].index];
			float total = 0.0f;
			for (size_t i = 1; i < len; ++i) {
				Vector3 next = terrain.vertices[river.vertices[i].index];
				total += (v3 - next).squareMagnitude();
				v3 = next;
			}
			return sqrtf(total / static_cast<float>(len << 5));
		}

		int addVert(int offset, const Vector3 &sink) {
			auto i = addedVerts.find(offset);
			if (i != addedVerts.end()) {
				return i->second;
			}
			int out = static_cast<int>(output->vertices.size());
			addedVerts.emplace(offset, out);
			output->vertices.push_back(terrain.vertices[offset] - sink);
			output->uv.push_back(uvs[offset]);
			return out;
		}

		void addTriangle(int offset, const Vector3 &sink) {
			if (addedTriangles.find(offset) != addedTriangles.end()) {
				return;
			}
			addedTriangles.emplace(offset, output->triangles.size());
			for (size_t i = 0; i != 3; ++i) {
				output->triangles.push_back(addVert(terrain.triangles[offset + i], sink));
			}
		}

		void reset(const River::Vertices &verts) {
			size_t len = verts.size();
			leftVerts.clear();
			rightVerts.clear();
			flows.resize(len);
			float maxFlow = 0.0f;
			for (size_t i = 0; i != len; ++i) {
				float flow = static_cast<float>(verts[i].flow);
				if (flow < maxFlow) {
					flow = maxFlow;
				}
				flows[i] = maxFlow = flow;
			}
			float minFlow = flows.front() * flowMul;
			maxFlow *= flowMul;
			float range = maxFlow - minFlow;
			if (range == 0) {
				minFlow = 0;
				range = maxFlow;
			}
			float flowAdjust = 1.0f / range;
			for (size_t i = 0; i != len; ++i) {
				float flow = flows[i] * flowMul;
				flows[i] = (flow - minFlow) * flowAdjust;
			}
		}

		int findClosestVertexToMidPoint(int a, int b, const MeshEdgeMap &mem) const{
			Vector3 mid = terrain.vertices[a] + terrain.vertices[b];
			int closest;
			float min = std::numeric_limits<float>::max();
			auto an = mem.vertex(a);
			for (auto i = mem.vertex(b); i.first != i.second; ++i.first) {
				int vert = *i.first;
				for (auto j = an.first; j != an.second; ++j) {
					if (vert == *j) {
						float dist = (terrain.vertices[vert] - mid).squareMagnitude();
						if (dist < min) {
							min = dist;
							closest = vert;
						}
					}
				}
			}
			return closest;
		}

		void addRiver(const River &river) {
			bool skipLast = river.join != -1;
			float depthMul = computeDepthMul(river);
			if (depthMul < FLT_EPSILON) {
				return;
			}
			reset(river.vertices);
			setWidths(river.vertices, skipLast);
			for (const auto &i : river.vertices) {
				squish(i.index);
			}
			assignLeftAndRight(river);
			smooth(river, leftVerts);
			smooth(river, rightVerts);
			smooth(river);
			setUVs(river.vertices);
			size_t len = river.vertices.size();
			for (size_t i = 0; i != len; ++i) {
				int idx = river.vertices[i].index;
				Vector3 sink = terrain.normals[idx] * (depthMul * flows[i] * 0.25f);
				for (auto j = mtm.vertex(idx); j.first != j.second; ++j.first) {
					addTriangle(*j.first, sink);
				}
			}
			if (skipLast) {
				--len;
			}
			for (size_t i = 0; i != len; ++i) {
				int idx = river.vertices[i].index;
				terrain.vertices[idx] -= terrain.normals[idx] * (depthMul * flows[i]);
			}
		}

		void fixJoins() {
			for (auto i : addedTriangles) {
				for (int j = 0; j != 3; ++j) {
					int vert = terrain.triangles[i.first + j];
					if (added.find(vert) == added.end()) {
						Vector3 &current = output->vertices[output->triangles[i.second + j]];
						const Vector3 &bank = terrain.vertices[vert];
						if (current.z > bank.z) {
							current.z = bank.z;
						}
						current.x = bank.x;
						current.y = bank.y;
					}
				}
			}
		}
		
		void normaliseUVs() {
			Vector2 min(1.0f, 1.0f);
			for (const Vector2 &uv : output->uv) {
				if (uv.x < min.x) {
					min.x = uv.x;
				}
				if (uv.y < min.y) {
					min.y = uv.y;
				}
			}
			for (Vector2 &uv : output->uv) {
				uv -= min;
			}
		}

		void fixDecoration(Decoration &decoration, const River::Vertices &verts) {
			for (auto i : leftVerts) {
				decoration.setRiver(i.first, true);
			}
			for (auto i : rightVerts) {
				decoration.setRiver(i.first, true);
			}
			for (auto i : verts) {
				decoration.setRiver(i.index, false);
			}
		}
	};

	float computeFlowMul(const Rivers::RiverList &rivers) {
		size_t len = rivers.size();
		int maxFlow = 0;
		for (const auto &i : rivers) {
			int flow = i->vertices.back().flow;
			if (flow > maxFlow) {
				maxFlow = flow;
			}
		}
		return 1.0f / static_cast<float>(maxFlow);
	}

	void addSortRiver(const Rivers::RiverList &rivers, std::vector<const River*> &output, std::unordered_set<int> &added, int offset) {
		const River *river = &*rivers[offset];
		int join = river->join;
		if (join != -1 && added.find(join) == added.end()) {
			addSortRiver(rivers, output, added, join);
		}
		added.insert(offset);
		output.push_back(river);
	}

	void sortRivers(const Rivers::RiverList &rivers, std::vector<const River*> &output) {
		size_t len = rivers.size();
		output.reserve(len);
		std::unordered_set<int> added(len);
		for (int i = 0, j = static_cast<int>(len); i != j; ++i){
			if (added.find(i) != added.end()) {
				continue;
			}
			addSortRiver(rivers, output, added, i);
		}
	}
}

MeshWithUV &motu::createRiverMesh(float uvScale, Decoration &decoration, const Rivers::RiverList &rivers, Mesh &terrain, MeshWithUV &output, const MeshEdgeMap &mem, const MeshTriangleMap &mtm) {
	MeshWithUV *addrOutput = &output;
	float maxFlow = computeFlowMul(rivers);
	//Very strange bug addrOutput gets nulled out when passed in here
	RiverMeshBuilder rmb(maxFlow, uvScale, addrOutput, terrain, mem, mtm);
	//but assigning it again here works
	rmb.output = addrOutput;
	std::vector<const River*> sorted;
	sortRivers(rivers, sorted);
	for (const auto i : sorted) {
		rmb.addRiver(*i);
		rmb.fixDecoration(decoration, i->vertices);
	}
	rmb.fixJoins();
	rmb.normaliseUVs();
	output.calculateNormals(output);
	return output;
}