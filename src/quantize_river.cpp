#include <queue>
#include <unordered_map>

#include "river_quantizer.h"
#include "bresenham.h"
#include "vector2int.h"

using namespace motu;

namespace {

	struct Coords {
		int x, y, flow;
		Coords(const Vector3 &v3, const Grid<float> &grid, int flow) :x(static_cast<int>(v3.x * grid.width())), y(static_cast<int>(v3.y * grid.height())), flow(flow) {}
	};

	struct Setter {
		QuantisedRiver &out;
		int lastFlow;

		Setter(QuantisedRiver &out) : out(out){}

		void operator()(int x, int y) {
			out.emplace_back(x, y, lastFlow);
		}
	};

	struct IntTriangle {
		int vertices[3];

		IntTriangle(int a, int b, int c) {
			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;
			std::sort(vertices, vertices + 3);
		}

		size_t hash() const{
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

		bool operator!=(const IntTriangle &other) const{
			for (int i = 0; i != 3; ++i) {
				if (vertices[i] == other.vertices[i]) {
					return false;
				}
			}
			return true;
		}
	};

	Vector2 &clamp(Vector2 &v2) {
		v2.x = roundf(v2.x);
		v2.y = roundf(v2.y);
		return v2;
	}

	struct MeshConstructor {
		std::unordered_map<Vector2, int, Hasher<Vector2>> added;
		std::unordered_set<Vector2, Hasher<Vector2>> ignore, smoothSet;
		std::vector<std::pair<int, int>> recurseList;
		std::vector<Vector3> vertices;
		std::vector<Vector2> uv;
		std::vector<IntTriangle> triangles;
		std::unordered_set<IntTriangle, Hasher<IntTriangle>> addedTriangles;
		Grid<float> &grid;

		MeshConstructor(Grid<float> &grid) : grid(grid) {}

		int get(const Vector2 &v2, float x, int y) {
			auto i = added.find(v2);
			if (i != added.end()) {
				return i->second;
			}
			else {
				int idx = static_cast<int>(vertices.size());
				added.emplace(v2, idx);
				vertices.emplace_back((v2.x + 0.5f), (v2.y + 0.5f), 0.0f);
				uv.emplace_back(x, static_cast<float>(y));
				return idx;
			}
		}

		float &getHeight(const Vector2 &v2) {
			int x = static_cast<int>(/*roundf(*/v2.x/*)*/);
			int y = static_cast<int>(/*roundf(*/v2.y/*)*/);
			int w = grid.width();
			if (x == w) {
				--x;
			}
			if (y == w) {
				--y;
			}
			return grid(x, y);
		}

		std::pair<int, int> getOffsets(const Vector2 &v2) {
			int x = static_cast<int>(/*roundf(*/v2.x/*)*/);
			int y = static_cast<int>(/*roundf(*/v2.y/*)*/);
			int w = grid.width();
			if (x == w) {
				--x;
			}
			if (y == w) {
				--y;
			}
			return std::make_pair(x, y);
		}

		void ensureDownness(const QuantisedRiver& path) {
			int x = path[0].x, y = path[0].y;
			float max = grid(x, y);
			for (size_t i = 1; i < path.size(); ++i) {
				int x = path[i].x, y = path[i].y;
				float &z = grid(x, y);
				if (z > max) {
					z = max;
				}
				else {
					max = z;
				}
			}
		}

		void smooth(const std::pair<int, int> &current, float height, bool recurse = true) {
			smoothSet.emplace(current.first, current.second);
			recurseList.clear();
			int xFrom = current.first == 0 ? current.first : current.first - 1, xTo = current.first + 1;
			if (xTo == grid.width()) {
				--xTo;
			}
			int yFrom = current.second == 0 ? current.second : current.second - 1, yTo = current.second + 1;
			if (yTo == grid.width()) {
				--yTo;
			}
			for (int y = yFrom; y <= yTo; ++y) {
				for (int x = xFrom; x <= xTo; ++x) {
					Vector2 pos(x, y);
					if (ignore.find(pos) != ignore.end()) {
						continue;
					}
					smoothSet.insert(pos);
					ignore.insert(pos);
					float z = grid(x, y);
					if (z > height) {
						grid(x, y) = (z + height) * 0.5f;
						if (recurse) {
							recurseList.emplace_back(x, y);
						}
					}
				}
			}
			if (!recurse) {
				return;
			}
			for (const std::pair<int, int> &i : recurseList) {
				smooth(i, grid(i.first, i.second), false);
			}
		}

		void smooth() {
			std::vector<int> left, right;
			left.reserve(uv.size());
			right.reserve(uv.size());
			for (int i = 0; i != uv.size(); ++i) {
				if (uv[i].x == 0.0f) {
					left.push_back(i);
				}
				else {
					right.push_back(i);
				}
			}
			std::vector<Vector3> adjusted(vertices);
			float third = 1.0f / 3.0f;
			for (int i = 1, last = left.size() - 1; i < last; ++i) {
				adjusted[left[i]] = (vertices[left[i - 1]] + vertices[left[i]] + vertices[left[i + 1]]) * third;
			}
			for (int i = 1, last = right.size() - 1; i < last; ++i) {
				adjusted[right[i]] = (vertices[right[i - 1]] + vertices[right[i]] + vertices[right[i + 1]]) * third;
			}
			vertices = adjusted;
		}

		void smoothRiverBed() {
			std::vector<Vector3> heightAdjustments;
			heightAdjustments.reserve(smoothSet.size());
			float scale = 1.0f / 9.0f;
			for (const Vector2 &v2 : smoothSet) {
				int x = static_cast<int>(v2.x), y = static_cast<int>(v2.y);
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
				heightAdjustments.emplace_back(v2.x, v2.y, total * scale);
			}
			for (const Vector3 &v3 : heightAdjustments) {
				grid(static_cast<int>(v3.x), static_cast<int>(v3.y)) = v3.z;
			}
		}

		void stretchMesh() {
			std::vector<std::pair<int, int>> axis(vertices.size(), std::make_pair(-1, -1));
			for (int i = 0; i != uv.size(); ++i) {
				const Vector2 &val = uv[i];
				if (val.x == 0.0f) {
					axis[static_cast<int>(val.y)].first = i;
				}
				else {
					axis[static_cast<int>(val.y)].second = i;
				}
			}
			for (const std::pair<int, int> &i : axis) {
				if (i.first == -1 || i.second == -1) {
					continue;
				}
				const Vector3 &a = vertices[i.first], &b = vertices[i.second];
				Vector3 direction = (b - a).normalized();
				vertices[i.first] -= direction;
				vertices[i.second] += direction;
			}
		}

		void setHeight(const Vector2 &current, const Vector2 &last, float height) {
			std::pair<int, int> c = getOffsets(current), l = getOffsets(last);
			grid(c.first, c.second) = height;
			ignore.clear();
			ignore.emplace(l.first, l.second);
			ignore.emplace(c.first, c.second);
			if (l.first == c.first || l.second == c.second) {
				smooth(c, height);
				return;
			}
			float a = grid(c.first, l.second), b = grid(l.first, c.second);
			if (height < a) {
				grid(c.first, l.second) = height;
			}
			if (height < b) {
				grid(l.first, c.second) = height;
			}
			smooth(c, height);
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

		MeshWithUV &copyToMesh(MeshWithUV &mesh) {
			mesh.vertices.reserve(vertices.size());
			std::copy(vertices.begin(), vertices.end(), std::back_inserter(mesh.vertices));
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
			return mesh;
		}

		float getSlope(const Vector2 &a, const Vector2 &b) {
			Vector3 up(a.x, a.y, getHeight(a) * grid.width());
			Vector3 down(b.x, b.y, getHeight(b) * grid.width());
			Vector3 dir = up - down;
			float len = dir.magnitude();
			return dir.z / len;
		}
	};
}

QuantisedRiver &motu::quantiseRiver(const Grid<float> &grid, const Island::VertexList &list, QuantisedRiver &out) {
	Coords last(list.front().vertex, grid, list.front().flow);
	Bresenham<Setter> bresenham(out);
	for (size_t i = 1; i < list.size(); ++i) {
		Coords current(list[i].vertex, grid, list[i].flow);
		bresenham.setter.lastFlow = list[i].flow;
		bresenham(last.x, last.y, current.x, current.y);
		out.pop_back();
		last = current;
	}
	out.emplace_back(last.x, last.y, last.flow);
	return out;
}

MeshWithUV &motu::createQuantisedRiverMesh(Grid<float> &grid, const QuantisedRiver& path, MeshWithUV &mesh) {
	float depth = 0.375f / static_cast<float>(grid.width());
	MeshConstructor mc(grid);
	mc.ensureDownness(path);
	Vector2 last(path[0].x, path[0].y);
	Vector2 current(path[1].x, path[1].y);
	Vector2 direction = (current - last).perp().normalized() * 3.0f;
	Vector2 lastLeft = clamp(last - direction), lastRight = clamp(last + direction);
	mc.get(lastLeft, 0.0f, 0);
	mc.get(lastRight, 2.0f, 0);
	int lastOffset = static_cast<int>(path.size()) - 1;
	for (int i = 1; i <= lastOffset; ++i) {
		current = Vector2(path[i].x, path[i].y);
		direction = (current - last).perp().normalized() * 3.0f;
		Vector2 left = clamp(current - direction), right = clamp(current + direction);
		Edge lastEdge(lastLeft, lastRight);
		int leftIdx, rightIdx;
		if (lastEdge.intersects(Edge(left, current))) {
			left = lastLeft;
			leftIdx = mc.get(left, 0.0f, i);
			rightIdx = mc.get(right, 2.0f, i);
			mc.add(leftIdx, mc.get(lastRight, 2.0f, i - 1), rightIdx);
		}
		else if (lastEdge.intersects(Edge(current, right))){
			right = lastRight;
			leftIdx = mc.get(left, 0.0f, i);
			rightIdx = mc.get(right, 2.0f, i);
			mc.add(leftIdx, mc.get(lastLeft, 0.0f, i - 1), rightIdx);
		}
		else {
			leftIdx = mc.get(left, 0.0f, i);
			rightIdx = mc.get(right, 2.0f, i);
			mc.add(leftIdx, mc.get(lastLeft, 0.0f, i - 1), mc.get(lastRight, 2.0f, i - 1));
			mc.add(leftIdx, mc.get(lastRight, 2.0f, i - 1), rightIdx);
		}
		float min = mc.getHeight(current);
		lastLeft = left;
		float z = mc.getHeight(left);
		if (z < min) {
			min = z;
		}
		lastRight = right;
		z = mc.getHeight(right);
		if (z < min) {
			min = z;
		}
		float carve = 1.0f - mc.getSlope(last, current);
		carve = carve * carve * depth;
		mc.vertices[rightIdx].z = (min - carve * 0.75f);
		mc.vertices[leftIdx].z = (min - carve * 0.75f);
		if (i != lastOffset && i != 1) {
			mc.setHeight(current, last, min - carve);
		}
		last = current;
	}
	mc.smooth();
	mc.smoothRiverBed();
	mc.ensureDownness(path);
	return mc.copyToMesh(mesh);
}

void motu::dedupeQuantisedRivers(std::vector<QuantisedRiver*> &rivers) {
	std::sort(rivers.begin(), rivers.end(), [](const QuantisedRiver *a, const QuantisedRiver *b) {
		return a->size() > b->size();
	});
	std::unordered_set<QuantisedRiverNode, Hasher<QuantisedRiverNode>> usedNodes;
	for (auto river : rivers) {
		for (int i = 0; i != river->size(); ++i) {
			if (usedNodes.find((*river)[i]) != usedNodes.end()) {
				river->resize(i + 1);
				break;
			}
			usedNodes.insert((*river)[i]);
		}
	}
}