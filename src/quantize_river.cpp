#include <queue>
#include <unordered_map>

#include "river_quantizer.h"
#include "bresenham.h"
#include "vector2int.h"
#include "bounding_box.h"
#include "brtree.h"

#ifndef MAX_RIVER_DEPTH
#define MAX_RIVER_DEPTH 3.0f
#endif
#ifndef TRIANGULATION_TLBR || TRIANGULATION_BLTR
#define TRIANGULATION_BLTR
#endif
#ifndef RIVER_UV_SCALE
#define RIVER_UV_SCALE 0.25f;
#endif
#ifndef RIVER_STRETCH
#define RIVER_STRETCH 2.0f;
#endif


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

	Vector2Int clamp(Vector2 &v2) {
		return Vector2Int(static_cast<int>(roundf(v2.x)), static_cast<int>(roundf(v2.y)));
	}

	struct EndBuilder{
		MeshWithUV &mesh;
		const MeshWithUV &join;
		int endVertA, endVertB;
		std::vector<int> in;

		EndBuilder(MeshWithUV &mesh, const MeshWithUV &join) : mesh(mesh), join(join){
		}

		bool findEndVertices() {
			size_t len = mesh.uv.size();
			endVertA = -1, endVertB = -1;
			float ha = std::numeric_limits<float>::min(), hb = std::numeric_limits<float>::min();
			for (size_t i = 0; i != len; ++i) {
				const Vector2 &uv = mesh.uv[i];
				if (uv.x < 0.0f) {
					if (uv.y > ha) {
						endVertA = i;
						ha = uv.y;
					}
				}
				else if (uv.y > hb) {
					endVertB = i;
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
			return out / added.size();
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
			centre /= in.size();
			centre += mesh.vertices[endVertA] + mesh.vertices[endVertB];
			centre /= 3.0f;
		}

		void addTriangle(int a, int b, int c) {
			if (!Triangle(mesh.vertices[a].asVector2(), mesh.vertices[b].asVector2(), mesh.vertices[c].asVector2()).isClockwise()) {
				std::swap(b, c);
			}
			mesh.triangles.push_back(a);
			mesh.triangles.push_back(b);
			mesh.triangles.push_back(c);
		}

		void completeCap(int centreOffset, int a, int b) {
			addTriangle(centreOffset, a, endVertA);
			addTriangle(centreOffset, b, endVertB);
			addTriangle(centreOffset, endVertA, endVertB);
		}

		void addRiverEnd(int centreOffset) {
			for (size_t i = 0; i != in.size(); ++i) {
				int replace = mesh.vertices.size();
				mesh.vertices.push_back(join.vertices[in[i]]);
				mesh.uv.push_back(join.uv[in[i]]);
				in[i] = replace;
			}
			for (size_t i = 1; i < in.size(); ++i) {
				int a = in[i - 1], b = in[i];
				if (!Triangle(mesh.vertices[centreOffset].asVector2(), mesh.vertices[a].asVector2(), mesh.vertices[b].asVector2()).isClockwise()) {
					std::swap(a, b);
				}
				mesh.triangles.push_back(centreOffset);
				mesh.triangles.push_back(a);
				mesh.triangles.push_back(b);
			}
			Vector2 av = mesh.vertices[in.front()].asVector2(), bv = mesh.vertices[in.back()].asVector2();
			Vector2 endA = mesh.vertices[endVertA].asVector2(), endB = mesh.vertices[endVertB].asVector2();
			Vector2 aa = av - endA, ab = av - endB, ba = bv - endA, bb = bv - endB;
			if (aa.sqrMagnitude() < ab.sqrMagnitude()) {
				if (bb.sqrMagnitude() <= ba.sqrMagnitude()) {
					completeCap(centreOffset, in.front(), in.back());
				}
			}
			else if (ba.sqrMagnitude() <= bb.sqrMagnitude()) {
				completeCap(centreOffset, in.back(), in.front());
			}
		}

		void createEndPiece(const std::vector<size_t> &joinTriangles) {
			if (!findEndVertices()) {
				return;
			}
			const Vector3 &va = mesh.vertices[endVertA], &vb = mesh.vertices[endVertB];
			std::unordered_set<int> addedA, addedB;
			addedA.reserve(joinTriangles.size() << 1);
			addedB.reserve(joinTriangles.size() << 1);
			for (size_t i : joinTriangles) {
				for (size_t j = 0; j != 3; ++j) {
					int offset = join.triangles[i + j];
					std::unordered_set<int> &set = join.uv[offset].x < 0.0f ? addedA : addedB;
					if (set.find(offset) == set.end()) {
						set.insert(offset);
					}
				}
			}
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
			int centreOffset = mesh.vertices.size();
			mesh.vertices.push_back(centre);
			mesh.uv.push_back(centreUv);
			addRiverEnd(centreOffset);
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
			if (verts.size() < 2) {
				return;
			}
			Vector2 current = vertices[verts[0]].asVector2(), next = vertices[verts[1]].asVector2();
			for (int i = 1, j = static_cast<int>(verts.size()) - 1; i < j; ++i) {
				Vector2 last = current;
				current = next;
				next = vertices[verts[i + 1]].asVector2();
				Vector2 dir = (last - next).perp().normalized() * direction * RIVER_STRETCH;
				dir += current;
				vertices[verts[i]].x = dir.x;
				vertices[verts[i]].y = dir.y;
			}
		}

		void smooth() {
			std::vector<Vector3> adjusted(vertices);
			float third = 1.0f / 3.0f;
			int last = leftVerts.size() - 1;
			for (int i = 1; i < last; ++i) {
				adjusted[leftVerts[i]] = (vertices[leftVerts[i - 1]] + vertices[leftVerts[i]] + vertices[leftVerts[i + 1]]) * third;
			}
			last = rightVerts.size() - 1;
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

		MeshWithUV &copyToMesh(MeshWithUV &sliced, float seaLevel) {
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
			mesh.slice(BoundingBox(0.0f, 0.0f, seaLevel, grid.width(), grid.height(), grid.width()), sliced);
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

		void clampEdges() {
			for (Vector3 &vert : vertices) {
#ifdef TRIANGULATION_TLBR
				float height = grid.interpTLBR(vert.x, vert.y);
#else
				float height = grid.interpTRBL(vert.x, vert.y);
#endif
				if (vert.z > height) {
					vert.z = height;
				}
			}
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

MeshWithUV &motu::createQuantisedRiverMesh(HeightMap &grid, const QuantisedRiver& path, MeshWithUV &mesh, float seaLevel, float maxFlow) {
	float depth = 0.2f / static_cast<float>(grid.width());
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
		mc.vertices[rightIdx].z = (min - carve * 0.25f);
		mc.vertices[leftIdx].z = (min - carve * 0.25f);
		if (i != lastOffset && i != 1) {
			mc.setHeight(current, last, min - carve, breadth - 1);
		}
		last = current;
	}
	mc.smooth();
	mc.smoothRiverBed();
	mc.ensureDownness(path);
	mc.clampEdges();
	return mc.copyToMesh(mesh, seaLevel);
}

void motu::dedupeQuantisedRivers(std::vector<QuantisedRiver*> &rivers, std::vector<int> &flowIntos) {
	flowIntos.insert(flowIntos.begin(), rivers.size(), -1);
	std::sort(rivers.begin(), rivers.end(), [](const QuantisedRiver *a, const QuantisedRiver *b) {
		return a->size() > b->size();
	});
	std::unordered_map<QuantisedRiverNode, int, Hasher<QuantisedRiverNode>> usedNodes;
	for (int j = 0; j != rivers.size(); ++j) {
		auto river = rivers[j];
		for (int i = 0; i != river->size(); ++i) {
			auto k = usedNodes.find((*river)[i]);
			if (k != usedNodes.end()) {
				river->resize(i + 1);
				flowIntos[j] = k->second;
				continue;
			}
			usedNodes.emplace((*river)[i], j);
		}
	}

}

void motu::joinQuantisedRiverMeshes(MeshWithUV &from, const MeshWithUV &to) {
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
	std::vector<size_t> glueTo;
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
						glueTo.push_back(*j.first);
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
	if (!glueTo.empty()) {
		EndBuilder endBuilder(from, to);
		endBuilder.createEndPiece(glueTo);
	}
	from.normals.clear();
	from.calculateNormals();
}