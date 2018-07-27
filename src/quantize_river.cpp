#include <queue>
#include <unordered_map>
#include <stack>
#include <array>

#include "river_quantizer.h"
#include "bresenham.h"
#include "vector2int.h"
#include "bounding_box.h"
#include "brtree.h"
#include "astar.h"

#ifndef MAX_RIVER_DEPTH
#define MAX_RIVER_DEPTH 2.0f
#endif
#if !defined(TRIANGULATION_TLBR) && !defined(TRIANGULATION_BLTR)
#define TRIANGULATION_BLTR
#endif
#ifndef RIVER_UV_SCALE
#define RIVER_UV_SCALE 0.25f
#endif
#ifndef RIVER_STRETCH
#define RIVER_STRETCH 1.0f
#endif
#ifndef MAX_RIVER_SLOPE
#define MAX_RIVER_SLOPE 0.35f
#endif

using namespace motu;

namespace {

	struct Coords {
		int x, y, flow;
		Coords(const Vector3 &v3, const Grid<float> &grid, int flow) :x(static_cast<int>(v3.x * grid.width())), y(static_cast<int>(v3.y * grid.height())), flow(flow) {}
	};

	struct Setter {
		QuantisedRiver &out;
		std::unordered_set<Vector2Int, Hasher<Vector2Int>> included;
		int lastFlow;

		Setter(QuantisedRiver &out) : out(out) {}

		void operator()(int x, int y) {
			Vector2Int v2(x, y);
			if (included.find(v2) == included.end()) {
				out.emplace_back(x, y, lastFlow);
				included.insert(v2);
			}
		}
	};

	void carveBack(HeightMap &heightMap, const QuantisedRiver &river, int from, float height){
		float current = heightMap(river[from].x, river[from].y);
		float shift = height - current;
		for (--from; from >= 0; --from) {
			heightMap(river[from].x, river[from].y) += shift;
			shift *= 0.75f;
		}
	}

	void flattenRiverJoins(HeightMap &heightMap, std::vector<QuantisedRiver*> &rivers, const std::vector<int> &flowIntos) {
		for (size_t i = 0; i != flowIntos.size(); ++i) {
			int target = flowIntos[i];
			if (target == -1) {
				continue;
			}
			const QuantisedRiver &inbound = *rivers[i];
			if (inbound.size() < 2) {
				continue;
			}
			QuantisedRiver &outbound = *rivers[target];
			const Vector2Int &join = inbound.back();
			const Vector2Int &in = inbound[inbound.size() - 2];
			for (size_t j = 1, k = outbound.size() - 1; j < k; ++j) {
				if (outbound[j] != join) {
					continue;
				}
				const Vector2Int &up = outbound[j - 1], &down = outbound[j + 1];
				float lowest = heightMap(in.x, in.y);
				float z = heightMap(down.x, down.y);
				if (z < lowest) {
					lowest = z;
				}
				carveBack(heightMap, inbound, static_cast<int>(inbound.size()) - 2, lowest);
				carveBack(heightMap, outbound, static_cast<int>(j) - 1, lowest);
				heightMap(join.x, join.y) = lowest;
				heightMap(in.x, in.y) = lowest;
				heightMap(up.x, up.y) = lowest;
				heightMap(down.x, down.y) = lowest;
				/*while (j != outbound.size()) {
					outbound[j++].flow += inbound.back().flow;
				}*/
				break;
			}
		}
	}

	struct IntTriangle {
		int vertices[3];

		IntTriangle(int a, int b, int c) {
			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;
			std::sort(vertices, vertices + 3);
		}

		size_t hash() const {
			return ((((vertices[0] * HASH_PRIME_A) + vertices[1]) * HASH_PRIME_B) + vertices[2]) * HASH_PRIME_C;
		}

		bool operator==(const IntTriangle &other) const {
			for (int i = 0; i != 3; ++i) {
				if (vertices[i] != other.vertices[i]) {
					return false;
				}
			}
			return true;
		}

		bool operator!=(const IntTriangle &other) const {
			for (int i = 0; i != 3; ++i) {
				if (vertices[i] == other.vertices[i]) {
					return false;
				}
			}
			return true;
		}
	};

	Vector2Int clamp(Vector2 &v2) {
		return Vector2Int(static_cast<int>(roundf(v2.x)), static_cast<int>(roundf(v2.y)));
	}

	struct EndBuilder {
		HeightMap &hm;
		MeshWithUV &mesh, &join;
		int endVertA, endVertB, centreIdx;
		std::vector<int> in;
		std::unordered_set<int> affected;
		std::vector<size_t> joinTriangles, meshTriangles;

		EndBuilder(HeightMap &hm, MeshWithUV &mesh, MeshWithUV &join) : hm(hm), mesh(mesh), join(join) {
		}

		static void addBedTriangles(std::unordered_map<int, int> &added, Mesh &bed, const std::vector<size_t> &offsets, const MeshWithUV &mesh) {
			for (size_t i : offsets) {
				for (size_t j = 0; j != 3; ++j) {
					int vert = mesh.triangles[i + j];
					auto k = added.find(vert);
					if (k == added.end()) {
						k = added.emplace_hint(k, vert, bed.vertices.size());
						bed.vertices.push_back(mesh.vertices[vert]);
					}
					bed.triangles.push_back(k->second);
				}
			}
		}

		typedef std::unordered_set<Vector2Int, Hasher<Vector2Int>> PixSet;

		struct EdgePixAdder {
			PixSet &ps;

			EdgePixAdder(PixSet &ps) : ps(ps) {}

			void operator()(int x, int y) {
				ps.emplace(x, y);
			}
		};

		static PixSet &getSmoothSet(const Mesh &mesh, PixSet &out) {
			EdgePixAdder adder(out);
			Bresenham<EdgePixAdder> bres(adder);
			Mesh::PerimeterSet ps;
			mesh.getPerimeterSet(ps);
			for (auto i : ps.edgeSet) {
				bres(
					static_cast<int>(mesh.vertices[i.first].x),
					static_cast<int>(mesh.vertices[i.first].y),
					static_cast<int>(mesh.vertices[i.second].x),
					static_cast<int>(mesh.vertices[i.second].y)
				);
			}
			return out;
		}

		void createPoolBed() {
			Mesh bed;
			std::unordered_map<int, int> added;
			addBedTriangles(added, bed, joinTriangles, join);
			added.clear();
			addBedTriangles(added, bed, meshTriangles, mesh);
			float xScale = 1.0f / hm.width(), yScale = 1.0f / hm.height();
			float deepest = bed.vertices.front().z - (yScale * 0.1f);
			for (Vector3 &vert : bed.vertices) {
				float depth = hm(
					static_cast<int>(vert.x),
					static_cast<int>(vert.y)
				);
				if (depth < deepest) {
					deepest = depth;
				}
			}
			for (Vector3 &vert : bed.vertices) {
				vert.z = deepest;
				vert.x *= xScale;
				vert.y *= yScale;
			}
			bed.rasterize(hm);
			PixSet ps;
			getSmoothSet(bed, ps);
			std::vector<std::pair<Vector2Int, float>> adjusted;
			adjusted.reserve(ps.size());
			static float diagWeight = 1.0f / sqrtf(2.0f);
			float total = 0.0f, count = 0.0f;
			for (const Vector2Int &pos : ps) {
				int bx = pos.x == 0 ? 0 : pos.x - 1, by = pos.y == 0 ? 0 : pos.y - 1, ex = pos.x + 1, ey = pos.y + 1;
				if (ex == hm.width()) {
					--ex;
				}
				if (ey == hm.height()) {
					--ey;
				}
				while (by <= ey) {
					for (int j = bx; j <= ex; ++j) {
						if (by != pos.y && j != pos.x) {
							count += diagWeight;
							total += hm(j, by) * diagWeight;
						}
						else {
							count += 1.0f;
							total += hm(j, by);
						}
					}
					++by;
				}
				adjusted.emplace_back(pos, total / count);
			}
			for (const auto &i : adjusted) {
				hm(i.first.x, i.first.y) = i.second;
			}
		}

		bool findEndVertices() {
			size_t len = mesh.uv.size();
			endVertA = -1, endVertB = -1;
			float ha = std::numeric_limits<float>::min(), hb = std::numeric_limits<float>::min();
			for (size_t i = 0; i != len; ++i) {
				const Vector2 &uv = mesh.uv[i];
				if (uv.x < 0.0f) {
					if (uv.y > ha) {
						endVertA = static_cast<int>(i);
						ha = uv.y;
					}
				}
				else if (uv.y > hb) {
					endVertB = static_cast<int>(i);
					hb = uv.y;
				}
			}
			return endVertA != -1 && endVertB != -1;
		}

		Vector2 computeCentroid(std::unordered_set<int> added) {
			Vector2 out = Vector2::zero();
			for (int i : added) {
				out += join.vertices[i].asVector2();
			}
			return out / static_cast<float>(added.size());
		}

		void sortByUv() {
			std::sort(in.begin(), in.end(), [this](int a, int b) {
				return join.uv[a].y < join.uv[b].y;
			});
		}

		void findHub(Vector3 &centre, Vector2 &uv) {
			centre = Vector3::zero();
			uv = Vector2::zero();
			for (int i : in) {
				centre += join.vertices[i];
				uv += join.uv[i];
			}
			centre /= static_cast<float>(in.size());
			centre += mesh.vertices[endVertA] + mesh.vertices[endVertB];
			centre /= 3.0f;
		}

		void addTriangle(int a, int b, int c) {
			if (!Triangle(mesh.vertices[a].asVector2(), mesh.vertices[b].asVector2(), mesh.vertices[c].asVector2()).isClockwise()) {
				std::swap(b, c);
			}
			meshTriangles.push_back(mesh.triangles.size());
			mesh.triangles.push_back(a);
			mesh.triangles.push_back(b);
			mesh.triangles.push_back(c);
		}

		void completeCap(int centreOffset, int a, int b) {
			addTriangle(centreOffset, a, endVertA);
			addTriangle(centreOffset, b, endVertB);
			addTriangle(centreOffset, endVertA, endVertB);
		}

		void addRiverEnd() {
			if (in.empty()) {
				return;
			}
			for (size_t i = 0; i != in.size(); ++i) {
				int replace = static_cast<int>(mesh.vertices.size());
				mesh.vertices.push_back(join.vertices[in[i]]);
				mesh.uv.push_back(join.uv[in[i]]);
				in[i] = replace;
			}
			for (size_t i = 1; i < in.size(); ++i) {
				int a = in[i - 1], b = in[i];
				if (!Triangle(mesh.vertices[centreIdx].asVector2(), mesh.vertices[a].asVector2(), mesh.vertices[b].asVector2()).isClockwise()) {
					std::swap(a, b);
				}
				meshTriangles.push_back(mesh.triangles.size());
				mesh.triangles.push_back(centreIdx);
				mesh.triangles.push_back(a);
				mesh.triangles.push_back(b);
			}
			Vector2 av = mesh.vertices[in.front()].asVector2(), bv = mesh.vertices[in.back()].asVector2();
			Vector2 endA = mesh.vertices[endVertA].asVector2(), endB = mesh.vertices[endVertB].asVector2();
			Vector2 aa = av - endA, ab = av - endB, ba = bv - endA, bb = bv - endB;
			if (aa.sqrMagnitude() < ab.sqrMagnitude()) {
				if (bb.sqrMagnitude() <= ba.sqrMagnitude()) {
					completeCap(centreIdx, in.front(), in.back());
				}
			}
			else if (ba.sqrMagnitude() <= bb.sqrMagnitude()) {
				completeCap(centreIdx, in.back(), in.front());
			}
			//unclampEdges();
		}

		void flattenJoinPool() {
			float deepest = mesh.vertices[endVertA].z < mesh.vertices[endVertB].z ? mesh.vertices[endVertA].z : mesh.vertices[endVertB].z;
			for (int i : affected) {
				float z = join.vertices[i].z;
				if (z < deepest) {
					deepest = z;
				}
			}
			mesh.vertices[endVertA].z = mesh.vertices[endVertB].z = deepest;
			for (int i : affected) {
				join.vertices[i].z = deepest;
			}
		}

		void createEndPiece() {
			if (!findEndVertices()) {
				return;
			}
			const Vector3 &va = mesh.vertices[endVertA], &vb = mesh.vertices[endVertB];
			std::unordered_set<int> addedA, addedB;
			addedA.reserve(joinTriangles.size() << 1);
			addedB.reserve(joinTriangles.size() << 1);
			affected.reserve(joinTriangles.size());
			for (size_t i : joinTriangles) {
				for (size_t j = 0; j != 3; ++j) {
					int offset = join.triangles[i + j];
					std::unordered_set<int> &set = join.uv[offset].x < 0.0f ? addedA : addedB;
					if (set.find(offset) == set.end()) {
						set.insert(offset);
						affected.insert(offset);
					}
				}
			}
			flattenJoinPool();
			Vector2 centroidA = computeCentroid(addedA), centroidB = computeCentroid(addedB);
			Vector2 endCentre = (va.asVector2() + vb.asVector2()) * 0.5f;
			if ((centroidA - endCentre).sqrMagnitude() < (centroidB - endCentre).sqrMagnitude()) {
				for (int i : addedA) {
					in.push_back(i);
				}
			}
			else {
				for (int i : addedB) {
					in.push_back(i);
				}
			}
			Vector3 centre;
			Vector2 centreUv;
			findHub(centre, centreUv);
			sortByUv();
			centreIdx = static_cast<int>(mesh.vertices.size());
			mesh.vertices.push_back(centre);
			mesh.uv.push_back(centreUv);
			addRiverEnd();
			createPoolBed();
		}
	};

	struct SteepSectionStripper {
		const HeightMap &hm;
		float maxSlope;
		std::unordered_set<Vector2Int, Hasher<Vector2Int>> removed;

		SteepSectionStripper(const HeightMap &hm) :hm(hm), maxSlope(MAX_RIVER_SLOPE / hm.height()) {}

		void erase(QuantisedRiver &river, size_t to) {
			for (size_t i = 0; i != to; ++i) {
				removed.emplace(river[i].x, river[i].y);
			}
			river.erase(river.begin(), river.begin() + to);
		}

		void removeIfTooSteep(QuantisedRiver &river) {
			for (size_t i = river.size() - 1; i > 0; --i) {
				const QuantisedRiverNode &node = river[i], &next = river[i - 1];
				if ((hm(next.x, next.y) - hm(node.x, node.y)) > maxSlope) {
					erase(river, i + 1);
					return;
				}
			}
		}

		void removeIfGoesUp(QuantisedRiver &river) {
			if (river.size() < 3) {
				erase(river, river.size());
				return;
			}
			const Vector2Int &secondToLast = river[river.size() - 2];
			if (hm(river.back().x, river.back().y) > hm(secondToLast.x, secondToLast.y)) {
				erase(river, river.size());
			}
		}

		void operator()(std::vector<QuantisedRiver*> &rivers, const std::vector<int> &flowIntos) {
			for (size_t i = 0; i != flowIntos.size(); ++i) {
				if (flowIntos[i] != -1) {
					removeIfGoesUp(*rivers[i]);
				}
			}
			float maxSlope = 0.5f / hm.height();
			for (QuantisedRiver *river : rivers) {
				if (!river->empty()) {
					removeIfTooSteep(*river);
				}
			}
			bool removing;
			do {
				removing = false;
				for (size_t i = 0; i != rivers.size(); ++i) {
					QuantisedRiver &river = *rivers[i];
					if (flowIntos[i] != -1 && !river.empty()) {
						if (removed.find(Vector2Int(river.back().x, river.back().y)) != removed.end()) {
							erase(river, river.size());
							removing = true;
						}
					}
				}
			} while (removing);
		}
	};

	struct MeshConstructor {
		std::unordered_map<Vector2Int, int, Hasher<Vector2Int>> added;
		std::unordered_set<Vector2Int, Hasher<Vector2Int>> ignore, smoothSet;
		std::vector<Vector3> vertices;
		std::vector<Vector2> uv;
		std::vector<IntTriangle> triangles;
		std::vector<int> leftVerts, rightVerts;
		std::unordered_set<IntTriangle, Hasher<IntTriangle>> addedTriangles;
		HeightMap &grid;
		float invMaxFlow;

		bool checkForInf() {
			for (const Vector3 &v3 : vertices) {
				if (isnan(v3.x) || isnan(v3.y)) {
					return true;
				}
			}
			return false;
		}

		MeshConstructor(HeightMap &grid, float maxFlow) : grid(grid), invMaxFlow(MAX_RIVER_DEPTH / maxFlow) {}

		int get(const Vector2Int &v2, bool left, float x, int y) {
			x *= RIVER_UV_SCALE;
			float uvy = static_cast<float>(y) * RIVER_UV_SCALE;
			auto i = added.find(v2);
			if (i != added.end()) {
				return i->second;
			}
			else {
				int idx = static_cast<int>(vertices.size());
				added.emplace(v2, idx);
				if (left) {
					leftVerts.emplace_back(idx);
					uv.emplace_back(x, uvy);
				}
				else {
					rightVerts.emplace_back(idx);
					uv.emplace_back(x, uvy);
				}
				vertices.emplace_back((v2.x + 0.5f), (v2.y + 0.5f), 0.0f);
				return idx;
			}
		}

		float &getHeight(const Vector2Int &v2) {
			int x = v2.x, y = v2.y, w = grid.width();
			if (x == w) {
				--x;
			}
			if (y == w) {
				--y;
			}
			return grid(x, y);
		}

		float &getHeight(const Vector2 &v2) {
			return getHeight(Vector2Int(v2.x, v2.y));
		}

		Vector2Int getOffsets(const Vector2 &v2) {
			Vector2Int out(v2.x, v2.y);
			int w = grid.width();
			if (out.x == w) {
				--out.x;
			}
			if (out.y == w) {
				--out.y;
			}
			return out;
		}

		void ensureDownness(const QuantisedRiver& path) {
			int lastX = path.front().x, lastY = path.front().y;
			float min = grid(lastX, lastY);
			for (size_t i = 1; i != path.size(); ++i) {
				int x = path[i].x, y = path[i].y;
				float *z = &grid(x, y);
				if (*z > min) {
					*z = min;
					Vector2Int topLeft, bottomRight;
					grid.assignNeighbourBounds(x, y, topLeft, bottomRight);
					while (topLeft.y <= bottomRight.y) {
						for (int ix = topLeft.x; ix <= bottomRight.x; ++ix) {
							if ((ix == x && topLeft.y == y) || (ix == lastX && topLeft.y == lastY)) {
								continue;
							}
							z = &grid(ix, topLeft.y);
							*z = ((*z * 3.0f) + min) * 0.25f;
						}
						++topLeft.y;
					}
				}
				else {
					min = *z;
				}
				if (lastX != x && lastY != y){
					z = &grid(lastX, y);
					if (*z > min) {
						*z = min;
					}
					z = &grid(x, lastY);
					if (*z > min) {
						*z = min;
					}
				}
				lastX = x;
				lastY = y;
			}
		}

		void smooth(const Vector2Int &current, float height, int recursions) {
			smoothSet.insert(current);
			std::vector<Vector2Int> recurseList;
			int xFrom = current.x == 0 ? current.x : current.x - 1, xTo = current.x + 1;
			if (xTo == grid.width()) {
				--xTo;
			}
			int yFrom = current.y == 0 ? current.y : current.y - 1, yTo = current.y + 1;
			if (yTo == grid.width()) {
				--yTo;
			}
			for (int y = yFrom; y <= yTo; ++y) {
				for (int x = xFrom; x <= xTo; ++x) {
					Vector2Int pos(x, y);
					if (ignore.find(pos) != ignore.end()) {
						continue;
					}
					smoothSet.insert(pos);
					ignore.insert(pos);
					float z = grid(x, y);
					if (z > height) {
						grid(x, y) = (z + height) * 0.5f;
						if (recursions > 0) {
							recurseList.emplace_back(x, y);
						}
					}
				}
			}
			if (recursions == 0) {
				return;
			}
			for (const Vector2Int &i : recurseList) {
				smooth(i, grid(i.x, i.y), recursions - 1);
			}
		}

		void stretchMesh(std::vector<int> verts, float direction) {
			if (verts.size() < 3) {
				return;
			}
			Vector2 current = vertices[verts[0]].asVector2(), next = vertices[verts[1]].asVector2();
			for (int i = 1, j = static_cast<int>(verts.size()) - 1; i < j; ++i) {
				Vector2 last = current;
				current = next;
				next = vertices[verts[i + 1]].asVector2();
				Vector2 dir = last - next;
				if (dir.sqrMagnitude() < FLT_EPSILON) {
					continue;
				}
				dir = (dir.perp().normalized() * direction * RIVER_STRETCH) + current;
				vertices[verts[i]].x = dir.x;
				vertices[verts[i]].y = dir.y;
			}
		}

		void smooth() {
			std::vector<Vector3> adjusted(vertices);
			static float third = 1.0f / 3.0f;
			int last = static_cast<int>(leftVerts.size()) - 1;
			for (int i = 1; i < last; ++i) {
				adjusted[leftVerts[i]] = (vertices[leftVerts[i - 1]] + vertices[leftVerts[i]] + vertices[leftVerts[i + 1]]) * third;
			}
			last = static_cast<int>(rightVerts.size()) - 1;
			for (int i = 1; i < last; ++i) {
				adjusted[rightVerts[i]] = (vertices[rightVerts[i - 1]] + vertices[rightVerts[i]] + vertices[rightVerts[i + 1]]) * third;
			}
			vertices = adjusted;
			stretchMesh(leftVerts, 1.0f);
			stretchMesh(rightVerts, -1.0f);
		}

		void smoothRiverBed() {
			std::vector<Vector3> heightAdjustments;
			heightAdjustments.reserve(smoothSet.size());
			float scale = 1.0f / 9.0f;
			for (const Vector2Int &v2 : smoothSet) {
				int x = v2.x, y = v2.y;
				int fromX = x == 0 ? 0 : x - 1;
				int toX = x + 1;
				if (toX == grid.width()) {
					--toX;
				}
				int fromY = y == 0 ? 0 : y - 1;
				int toY = y + 1;
				if (toY == grid.width()) {
					--toY;
				}
				float total = 0.0f;
				while (fromY <= toY) {
					for (x = fromX; x <= toX; ++x) {
						total += grid(x, fromY);
					}
					++fromY;
				}
				heightAdjustments.emplace_back(static_cast<float>(v2.x), static_cast<float>(v2.y), total * scale);
			}
			for (const Vector3 &v3 : heightAdjustments) {
				grid(static_cast<int>(v3.x), static_cast<int>(v3.y)) = v3.z;
			}
		}

		int getAdjustedFlow(int flow) {
			return static_cast<int>(sqrtf(static_cast<float>(flow)) * invMaxFlow);
		}

		void setHeight(const Vector2 &current, const Vector2 &last, float height, int smoothings) {
			Vector2Int c = getOffsets(current), l = getOffsets(last);
			grid(c.x, c.y) = height;
			ignore.clear();
			ignore.insert(l);
			ignore.insert(c);
			if (l.x == c.x || l.y == c.y) {
				smooth(c, height, smoothings);
				return;
			}
			float a = grid(c.x, l.y), b = grid(l.x, c.y);
			if (height < a) {
				grid(c.x, l.y) = height;
			}
			if (height < b) {
				grid(l.x, c.y) = height;
			}
			smooth(c, height, smoothings);
		}

		void add(int a, int b, int c) {
			if (a == b || a == c || b == c) {
				return;
			}
			IntTriangle tri(a, b, c);
			if (addedTriangles.find(tri) == addedTriangles.end()) {
				triangles.push_back(tri);
				addedTriangles.insert(tri);
			}
		}

		std::pair<size_t, size_t> findClosest(const Vector3 &vert, float &shortest, float &secondShortest) const{
			shortest = std::numeric_limits<float>::max();
			secondShortest = shortest;
			size_t closest = 0, secondClosest = 0;
			for (size_t i = 0; i != vertices.size(); ++i) {
				float dist = (vertices[i] - vert).squareMagnitude();
				if (dist < shortest) {
					secondShortest = shortest;
					secondClosest = closest;
					shortest = dist;
					closest = i;
					continue;
				}
				else if (dist < secondShortest) {
					secondShortest = dist;
					secondClosest = i;
				}
			}
			shortest = sqrtf(shortest);
			secondShortest = sqrtf(secondShortest);
			return std::make_pair(closest, secondClosest);
		}

		Vector2 interpUv(const Vector3 &pos) const{
			float distA, distB;
			std::pair<size_t, size_t> closest = findClosest(pos, distA, distB);
			float total = distA + distB;
			return (uv[closest.first] * (total - distA) + uv[closest.second] * (total - distB)) / total;
		}

		MeshWithUV &copyToMesh(MeshWithUV &sliced) {
			MeshWithUV mesh;
			std::unordered_map<Vector3, int, Hasher<Vector3>> lookup;
			lookup.reserve(vertices.size());
			mesh.vertices.reserve(vertices.size());
			for (int i = 0, j = static_cast<int>(vertices.size()); i != j; ++i) {
				lookup.emplace(vertices[i], i);
				mesh.vertices.push_back(vertices[i]);
			}
			mesh.uv = uv;
			mesh.triangles.reserve(triangles.size() * 3);
			for (IntTriangle tri : triangles) {
				int a = tri.vertices[0], b = tri.vertices[1], c = tri.vertices[2];
				Triangle test(vertices[a].asVector2(), vertices[b].asVector2(), vertices[c].asVector2());
				if (!test.isClockwise()) {
					std::swap(a, c);
				}
				mesh.triangles.push_back(a);
				mesh.triangles.push_back(b);
				mesh.triangles.push_back(c);
			}
			mesh.normals.reserve(mesh.vertices.size());
			for (int i = 0; i != mesh.vertices.size(); ++i) {
				mesh.normals.emplace_back(0.0f, 0.0f, 1.0f);
			}
			mesh.slice(BoundingBox(
				0.0f, 0.0f, grid.seaLevel(),
				static_cast<float>(grid.width()),
				static_cast<float>(grid.height()),
				static_cast<float>(grid.width()))
				, sliced);
			sliced.uv.reserve(sliced.vertices.size());
			for (int i = 0, j = static_cast<int>(sliced.vertices.size()); i != j; ++i) {
				auto k = lookup.find(sliced.vertices[i]);
				if (k == lookup.end()) {
					sliced.uv.push_back(interpUv(sliced.vertices[i]));
				}
				else {
					sliced.uv.push_back(mesh.uv[k->second]);
				}
			}
			return mesh;
		}

		float getSlope(const Vector2 &a, const Vector2 &b) {
			Vector3 up(a.x, a.y, getHeight(a) * grid.width());
			Vector3 down(b.x, b.y, getHeight(b) * grid.width());
			Vector3 dir = up - down;
			float len = dir.magnitude();
			return dir.z / len;
		}

		struct Clamper {
			float minHeight;
			HeightMap &heightMap;

			Clamper(HeightMap &heightMap, float minHeight) : heightMap(heightMap), minHeight(minHeight) {

			}

			void operator()(int x, int y) {
				int maxX = x + 1;
				if (maxX == heightMap.width()) {
					--maxX;
				}
				int maxY = y + 1;
				if (maxY == heightMap.height()) {
					--maxX;
				}
				while (y <= maxY) {
					for (int i = x; i <= maxX; ++i) {
						float height = heightMap(i, y);
						if (height < minHeight) {
							heightMap(i, y) = minHeight;
						}
					}
					++y;
				}
			}
		};

		void clampEdges(const std::vector<int> &side) {
			for (size_t i = 1, j = side.size() - 1; i < j; ++i){
				const Vector3 &a = vertices[side[i - 1]], &b = vertices[side[i]];
				Clamper clamper(grid, std::max(a.z, b.z));
				Bresenham<Clamper> bresenham(clamper);
				bresenham(
					static_cast<int>(a.x),
					static_cast<int>(a.y),
					static_cast<int>(b.x),
					static_cast<int>(b.y)
				);
			}
		}

		void clampEdges() {
			clampEdges(leftVerts);
			clampEdges(rightVerts);
		}
	};

	struct SeaFinder {
		typedef Vector2Int NodeType;
		typedef float CostType;
		typedef std::vector<Vector2Int> NeighboursType;

		NeighboursType mNeighbours;
		const HeightMap &heightMap;
		const Grid<bool> &seaMap;
		std::unordered_set<Vector2Int, Hasher<Vector2Int>> mVisited;

		SeaFinder(const HeightMap &heightMap, const Grid<bool> &seaMap) : heightMap(heightMap), seaMap(seaMap) {
			mNeighbours.reserve(8);
		}

		void clear() {
			mVisited.clear();
		}

		void visit(const Vector2Int &pos) {
			mVisited.insert(pos);
		}

		bool visited(const Vector2Int &pos) const{
			return mVisited.find(pos) != mVisited.end();
		}

		bool goal(const Vector2Int &pos) const {
			return seaMap(pos.x, pos.y);
		}

		float hueristic(const Vector2Int &coord) {
			return heightMap(coord.x, coord.y);
		}

		const NeighboursType &neighbours(const Vector2Int &coord) {
			mNeighbours.clear();
			int fromX = coord.x == 0 ? 0 : coord.x - 1, fromY = coord.y == 0 ? 0 : coord.y - 1;
			int toX = coord.x + 1, toY = coord.y + 1;
			if (toX == heightMap.width()) {
				--toX;
			}
			if (toY == heightMap.height()) {
				--toY;
			}
			while (fromY <= toY) {
				for (int x = fromX; x <= toX; ++x) {
					if (x == coord.x && fromY == coord.y) {
						continue;
					}
					mNeighbours.emplace_back(x, fromY);
				}
				++fromY;
			}
			return mNeighbours;
		}
	};
}

void RiverQuantiser::populateSeaMap() {
	int width = heightMap.width(), height = heightMap.height();
	int size = width * height;
	std::fill(seaMap.data(), seaMap.data() + size, false);
	for (int i = 0; i != width; ++i) {
		seaMap(i, 0) = true;
		seaMap(i, height - 1) = true;
	}
	for (int i = 0; i != height; ++i) {
		seaMap(0, i) = true;
		seaMap(width - 1, i) = true;
	}
	std::vector<Vector2Int> fringe;
	fringe.reserve(size);
	fringe.emplace_back(1, 1);
	seaMap(1, 1) = true;
	int seaCount = ((width + height) << 1) + 1;
	while (!fringe.empty()) {
		Vector2Int current = fringe.back();
		fringe.pop_back();
		for (int x = current.x - 1; x != current.x + 3; x += 2) {
			if (heightMap(x, current.y) < heightMap.seaLevel() && !seaMap(x, current.y)) {
				seaMap(x, current.y) = true;
				fringe.emplace_back(x, current.y);
			}
		}
		for (int y = current.y - 1; y != current.y + 3; y += 2) {
			if (heightMap(current.x, y) < heightMap.seaLevel() && !seaMap(current.x, y)) {
				seaMap(current.x, y) = true;
				fringe.emplace_back(current.x, y);
			}
		}
	}
}

QuantisedRiver &RiverQuantiser::quantiseRiver(const Island::VertexList &list, QuantisedRiver &out) {
	Coords last(list.front().vertex, heightMap, list.front().flow);
	Bresenham<Setter> bresenham(out);
	for (size_t i = 1; i < list.size(); ++i) {
		Coords current(list[i].vertex, heightMap, list[i].flow);
		bresenham.setter.lastFlow = list[i].flow;
		bresenham(last.x, last.y, current.x, current.y);
		//out.pop_back();
		last = current;
	}
	if (!out.empty()) {
		out.pop_back();
		out.emplace_back(last.x, last.y, last.flow);
		if (!seaMap(last.x, last.y)) {
			std::vector<Vector2Int> path;
			SeaFinder seaFinder(heightMap, seaMap);
			AStar<SeaFinder> astar(seaFinder);
			astar.search(Vector2Int(last.x, last.y), path);
			for (size_t i = 1; i < path.size(); ++i) {
				const Vector2Int &v2 = path[i];
				out.emplace_back(v2.x, v2.y, last.flow);
			}
		}
	}
	return out;
}

MeshWithUV &motu::createQuantisedRiverMesh(HeightMap &grid, const QuantisedRiver& path, MeshWithUV &mesh, float maxFlow) {
	float depth = 0.35f / static_cast<float>(grid.width());
	MeshConstructor mc(grid, maxFlow);
	mc.ensureDownness(path);
	Vector2 last(path[0].x, path[0].y);
	Vector2 current(path[1].x, path[1].y);
	Vector2 direction = (current - last).perp().normalized() * 3.0f;
	Vector2Int lastLeft = clamp(last - direction), lastRight = clamp(last + direction);
	int lastLeftIdx = mc.get(lastLeft, true, -1.5f, 0), lastRightIdx = mc.get(lastRight, false, 1.5f, 0);
	int lastOffset = static_cast<int>(path.size()) - 1;
	for (int i = 1; i <= lastOffset; ++i) {
		current = Vector2(path[i].x, path[i].y);
		int breadth = mc.getAdjustedFlow(path[i].flow);
		float breadthf = static_cast<float>(breadth);
		direction = (current - last).perp().normalized() * breadthf;
		Vector2Int left = clamp(current - direction), right = clamp(current + direction);
		Edge lastEdge(Vector2(lastLeft.x, lastLeft.y), Vector2(lastRight.x, lastRight.y));
		int leftIdx, rightIdx;
		float uvOff = breadthf + 1.0f;
		if (lastEdge.intersects(Edge(Vector2(left.x, left.y), Vector2(current.x, current.y)))) {
			left = lastLeft;
			leftIdx = lastLeftIdx;
			rightIdx = mc.get(right, false, uvOff, i);
			mc.add(leftIdx, lastRightIdx, rightIdx);
		}
		else if (lastEdge.intersects(Edge(Vector2(current.x, current.y), Vector2(right.x, right.y)))){
			right = lastRight;
			leftIdx = mc.get(left, true, -uvOff, i);
			rightIdx = lastRightIdx;
			mc.add(leftIdx, lastLeftIdx, rightIdx);
		}
		else {
			leftIdx = mc.get(left, true, -uvOff, i);
			rightIdx = mc.get(right, false, uvOff, i);
			mc.add(leftIdx, lastLeftIdx, lastRightIdx);
			mc.add(leftIdx, lastRightIdx, rightIdx);
		}
		float min = mc.getHeight(current);
		lastLeft = left;
		lastLeftIdx = leftIdx;
		float z = mc.getHeight(left);
		if (z < min) {
			min = z;
		}
		lastRight = right;
		lastRightIdx = rightIdx;
		z = mc.getHeight(right);
		if (z < min) {
			min = z;
		}
		float carve = /*(1.0f - mc.getSlope(last, current)) * */(breadthf + 1.0f) * depth;
		mc.vertices[rightIdx].z = (min - carve * 0.35f);
		mc.vertices[leftIdx].z = (min - carve * 0.35f);
		if (i != lastOffset && i != 1) {
			mc.setHeight(current, last, min - carve, breadth - 1);
		}
		last = current;
	}
	mc.smooth();
	mc.smoothRiverBed();
	mc.ensureDownness(path);
	return mc.copyToMesh(mesh);
}

void motu::dedupeQuantisedRivers(HeightMap &grid, std::vector<QuantisedRiver*> &rivers, std::vector<int> &flowIntos) {
	//LocalMinimaEnforcer enforce(grid);
	flowIntos.insert(flowIntos.begin(), rivers.size(), -1);
	SteepSectionStripper stripper(grid);
	std::sort(rivers.begin(), rivers.end(), [&grid](const QuantisedRiver *a, const QuantisedRiver *b) {
		Vector2Int va(a->back().x, a->back().y), vb(b->back().x, b->back().y);
		return grid(va.x, va.y) < grid(vb.x, vb.y);
	});
	std::unordered_map<QuantisedRiverNode, int, Hasher<QuantisedRiverNode>> usedNodes;
	for (int j = 0; j != rivers.size(); ++j) {
		auto river = rivers[j];
		for (int i = 0; i != river->size(); ++i) {
			auto k = usedNodes.find((*river)[i]);
			if (k != usedNodes.end()) {
				river->resize(i + 1);
				if (flowIntos[j] == -1) {
					flowIntos[j] = k->second;

				}
				else {
					//TODO: Big bug causes 3 way join, must sort out properly some day
					river->resize(0);
				}
				continue;
			}
			usedNodes.emplace((*river)[i], j);
		}
	}
	stripper(rivers, flowIntos);
	//flattenRiverJoins(grid, rivers, flowIntos);
	/*for (QuantisedRiver *river : rivers) {
		if (!river->empty()) {
			enforce.clear();
			enforce(*river);
		}
	}*/
}

void motu::joinQuantisedRiverMeshes(HeightMap &hm, MeshWithUV &from, MeshWithUV &to) {
	typedef BRTree<BoundingRectangle, size_t> Tree;
	Tree tree(to.triangles.size() / 3);
	for (size_t i = 2; i < to.triangles.size(); i += 3) {
		Triangle tri(
			to.vertices[to.triangles[i - 2]].asVector2(),
			to.vertices[to.triangles[i - 1]].asVector2(),
			to.vertices[to.triangles[i]].asVector2()
		);
		tree.add(tri.boundingRectangle(), i - 2);
	}
	Tree::Values values;
	std::vector<Triangle> addTriangles;
	std::vector<int> collapsedList;
	collapsedList.reserve(from.triangles.size());
	std::vector<int> included(from.vertices.size(), -1);
	EndBuilder endBuilder(hm, from, to);
	int offset = 0;
	for (size_t i = 2; i < from.triangles.size(); i += 3) {
		Triangle tri(
			from.vertices[from.triangles[i - 2]].asVector2(),
			from.vertices[from.triangles[i - 1]].asVector2(),
			from.vertices[from.triangles[i]].asVector2()
		);
		auto j = tree.intersecting(tri.boundingRectangle(), values);
		bool intersected = j.first != j.second;
		if (intersected) {
			Edge edges[3];
			tri.getEdges(edges);
			do {
				Triangle test(
					to.vertices[to.triangles[*j.first]].asVector2(),
					to.vertices[to.triangles[*j.first + 1]].asVector2(),
					to.vertices[to.triangles[*j.first + 2]].asVector2()
				);
				for (size_t k = 0; k != 3; ++k) {
					if (test.intersects(edges[k])) {
						endBuilder.joinTriangles.push_back(*j.first);
						intersected = true;
						break;
					}
				}
				++j.first;
			} while (j.first != j.second);
		}
		if (!intersected) {
			for (size_t k = i - 2; k <= i; ++k) {
				int vert = from.triangles[k];
				collapsedList.push_back(vert);
				if (included[vert] == -1) {
					included[vert] = offset++;
				}
			}
		}
	}
	std::vector<Vector3> newVerts;
	std::vector<Vector2> newUvs;
	newVerts.reserve(from.vertices.size());
	newUvs.reserve(from.uv.size());
	for (size_t i = 0; i != included.size(); ++i) {
		if (included[i] != -1) {
			newVerts.push_back(from.vertices[i]);
			newUvs.push_back(from.uv[i]);
		}
	}
	for (size_t i = 0; i != collapsedList.size(); ++i) {
		collapsedList[i] = included[collapsedList[i]];
	}
	from.vertices = newVerts;
	from.uv = newUvs;
	from.triangles = collapsedList;
	if (!endBuilder.joinTriangles.empty()) {
		endBuilder.createEndPiece();
	}
	from.normals.clear();
	from.calculateNormals();
}