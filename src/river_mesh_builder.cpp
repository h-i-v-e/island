#include <unordered_map>
#include <algorithm>

#include "river_mesh_builder.h"
#include "rivers.h"
#include "mesh.h"
#include "mesh_triangle_map.h"

using namespace motu;

namespace {
	struct RiverMeshBuilderImpl : public RiverMeshBuilder{

		RiverMeshBuilderImpl(float flowMul, Mesh &terrain, Mesh &ouput, const MeshEdgeMap &mem, const MeshTriangleMap &mtm) : RiverMeshBuilder(flowMul, terrain, output, mem, mtm){
		}

		Vector3 smooth(int vert, const std::unordered_set<int> &side) {
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

		void smooth(const River &river, const std::unordered_set<int> &side) {
			std::vector<std::pair<int, Vector3>> adjusted(side.size());
			size_t offset = 0;
			for (int i : side) {
				adjusted[offset++] = std::make_pair(i, smooth(i, side));
			}
			for (const auto &i : adjusted) {
				terrain.vertices[i.first] = i.second;
			}
		}

		Vector3 smooth(int vert, const MeshEdgeMap &mem) {
			Vector3 output = terrain.vertices[vert];
			auto i = mem.vertex(vert);
			int count = (i.second - i.first) + 1;
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

		void assignLeftAndRight(int current, int last) {
			Vector3 centre = terrain.vertices[current];
			Vector3 dir = terrain.vertices[last] - centre;
			for (auto i = mem.vertex(current); i.first != i.second; ++i.first) {
				int vert = *i.first;
				if (added.find(vert) != added.end()) {
					continue;
				}
				if ((terrain.vertices[vert] - centre).cross(dir).z < 0.0f) {
					leftVerts.insert(vert);
				}
				else {
					rightVerts.insert(vert);
				}
			}
		}

		void assignLeftAndRight(const River &river) {
			int last = river.vertices[0].index;
			for (size_t i = 1, j = river.vertices.size(); i < j; ++i) {
				int next = river.vertices[i].index;
				assignLeftAndRight(next, last);
				last = next;
			}
		}

		void moveFromTo(int move, const Vector3 &to, float dist) {
			Vector3 &target = terrain.vertices[move];
			target += (to - target) * dist;
		}

		void setWidth(int last, int current, int next, float flow) {
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

		void setWidths(const River::Vertices &verts) {
			for (size_t i = 1, j = verts.size() - 1; i < j; ++i) {
				setWidth(verts[i - 1].index, verts[i].index, verts[i + 1].index, 1.0f - flows[i]);
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
			return sqrtf(total / static_cast<float>(len << 4));
		}

		int addVert(int offset) {
			auto i = addedVerts.find(offset);
			if (i != addedVerts.end()) {
				return i->second;
			}
			int out = static_cast<int>(output.vertices.size());
			addedVerts.emplace(offset, out);
			output.vertices.push_back(terrain.vertices[offset]);
			return out;
		}

		void addTriangle(int offset) {
			if (addedTriangles.find(offset) != addedTriangles.end()) {
				return;
			}
			addedTriangles.insert(offset);
			for (size_t i = 0; i != 3; ++i) {
				output.triangles.push_back(addVert(terrain.triangles[offset + i]));
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
	};
}

void RiverMeshBuilder::smoothRiverBeds(const Rivers::RiverList &rivers) {
	RiverMeshBuilderImpl &impl = *reinterpret_cast<RiverMeshBuilderImpl*>(this);
	motu::tesselate(terrain, added);
	MeshEdgeMap mem(terrain);
	std::vector<std::pair<int, Vector3>> adjustments;
	for (const auto &river : rivers) {
		for (size_t i = 1, j = river->vertices.size(); i < j; ++i) {
			int vert = river->vertices[i - 1].index;
			adjustments.emplace_back(vert, impl.smooth(vert, mem));
			int next = river->vertices[i].index;
			adjustments.emplace_back(next, impl.smooth(next, mem));
			int middle = impl.findClosestVertexToMidPoint(vert, next, mem);
			adjustments.emplace_back(middle, impl.smooth(middle, mem));
		}
	}
	for (const auto &i : adjustments) {
		terrain.vertices[i.first] = i.second;
	}
}

void RiverMeshBuilder::addRiver(const River &river) {
	RiverMeshBuilderImpl &impl = *reinterpret_cast<RiverMeshBuilderImpl*>(this);
	float depthMul = impl.computeDepthMul(river);
	if (depthMul < FLT_EPSILON) {
		return;
	}
	impl.reset(river.vertices);
	impl.setWidths(river.vertices);
	for (const auto &i : river.vertices) {
		impl.squish(i.index);
	}
	impl.assignLeftAndRight(river);
	impl.smooth(river, leftVerts);
	impl.smooth(river, rightVerts);
	impl.smooth(river);
	for (const auto &i : river.vertices) {
		for (auto j = mtm.vertex(i.index); j.first != j.second; ++j.first) {
			impl.addTriangle(*j.first);
		}
	}
	for (size_t i = 0, j = river.vertices.size(); i != j; ++i){
		int idx = river.vertices[i].index;
		terrain.vertices[idx] -= terrain.normals[idx] * (depthMul * flows[i]);
	}
}