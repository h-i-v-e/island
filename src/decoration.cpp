#include "decoration.h"
#include "mesh.h"
#include "z_axis_collider.h"
#include "mesh_triangle_map.h"
#include <unordered_map>

using namespace motu;

namespace {
	float getRockChance(const Mesh &mesh, const MeshEdgeMap &mep, int offset) {
		Vector3 up = Vector3::unitZ();
		float total = mesh.normals[offset].dot(up);
		int count = 1;
		auto neighbours = mep.vertex(offset);
		while (neighbours.first != neighbours.second) {
			total += mesh.normals[*neighbours.first++].dot(up);
			++count;
		}
		return (1.0f - (total / static_cast<float>(count)));
	}

	std::vector<Vector3> &copyOutVertices(const Mesh &in, const std::vector<int> &ints, std::vector<Vector3> &out) {
		size_t len = ints.size();
		out.resize(len);
		for (size_t i = 0; i != len; ++i) {
			out[i] = in.vertices[ints[i]];
		}
		return out;
	}

	bool isLocalMaximum(const Mesh &mesh, const MeshEdgeMap &mep, int offset) {
		float z = mesh.vertices[offset].z;
		auto neighbours = mep.vertex(offset);
		while (neighbours.first != neighbours.second) {
			if (mesh.vertices[*neighbours.first++].z >= z) {
				return false;
			}
		}
		return true;
	}

	void setZAxis(const ZAxisCollider &collider, std::vector<Vector3> &verts) {
		int offset;
		int totalMiss = 0, miss = 0, hit = 0;
		for (Vector3 &vert : verts) {
			float z = collider.heightAt(vert.asVector2(), offset);
			if (offset != -1) {
				vert.z = z;
			}
		}
		//std::cout << "Hits: " << hit << ", Total Miss: " << totalMiss << ", Miss: " << miss << std::endl;
	}

	struct MeshCopier {
		std::unordered_map<int, int> feet, translate;
		const Mesh &from;
		MeshWithUV &to;
		float height, pertDist;
		Mesh::PerimeterSet ps;

		MeshCopier(const Mesh &from, MeshWithUV &to, float height) : translate(from.vertices.size()), from(from),
			to(to), height(height), pertDist(0.1f / sqrtf(from.vertices.size())){
		}

		int addVertex(int vert) {
			auto i = translate.find(vert);
			if (i != translate.end()) {
				return i->second;
			}
			int next = static_cast<int>(to.vertices.size());
			const Vector3 &v3 = from.vertices[vert];
			to.vertices.emplace_back(v3.x, v3.y, v3.z + height);
			translate.emplace(vert, next);
			return next;
		}

		void addTriangle(int triangle) {
			for (int i = 0; i != 3; ++i) {
				to.triangles.push_back(addVertex(from.triangles[triangle + i]));
			}
		}

		void addTriangle(int a, int b, int c) {
			auto &t = to.triangles;
			t.push_back(a);
			t.push_back(b);
			t.push_back(c);
		}

		bool loopDanger(int a, int b) {
			bool touched = false;
			for (int i = 0; i != 3; ++i) {
				int vert = to.triangles[a + i];
				for (int j = 0; j != 3; ++j) {
					if (vert == to.triangles[b + j]) {
						if (touched) {
							return false;
						}
						else {
							touched = true;
						}
					}
				}
			}
			return touched;
		}

		int getVertOffset(int tri, int vert, int add) {
			for (int i = 0;; ++i) {
				if (to.triangles[tri + i] == vert) {
					return to.triangles[tri + ((i + add) % 3)];
				}
			}
		}

		void avertLoops() {
			to.getPerimeterSet(ps);
			MeshTriangleMap mtm(to);
			for (int i : ps) {
				auto j = mtm.vertex(i);
				if (j.second - j.first == 2 && loopDanger(*j.first, *(j.first + 1))) {
					addTriangle(
						i,
						getVertOffset(*j.first, i, 1),
						getVertOffset(*(j.first + 1), i, 2)
					);
				}
			}
			ps.clear();
		}

		int addFoot(int head) {
			auto i = feet.find(head);
			if (i != feet.end()) {
				return i->second;
			}
			int out = to.vertices.size();
			const Vector3 &h = to.vertices[head];
			to.vertices.emplace_back(h.x, h.y, h.z - height);
			feet.emplace(head, out);
			return out;
		}

		static size_t getNeighbourCount(const MeshTriangleMap &mtm, int i) {
			auto j = mtm.vertex(i);
			return j.second - j.first;
		}

		static int leastNeighbours(const MeshTriangleMap &mtm, int a, int &b) {
			size_t lenA = getNeighbourCount(mtm, a), lenB = getNeighbourCount(mtm, b);
			if (lenA > lenB) {
				std::swap(a, b);
			}
			return a;
		}

		Vector3 getNormal(int a, int b, int c) const{
			return Triangle3(to.vertices[a], to.vertices[b], to.vertices[c]).normal();
		}

		void addTriangle(int a, int b, int c, const Vector3 &face) {
			float dir = (to.vertices[a] - face).dot(getNormal(a, b, c));
			if (dir > 0.0f) {
				addTriangle(a, b, c);
			}
			else {
				addTriangle(c, b, a);
			}
		}

		bool neighbourOfNeighbour(const MeshEdgeMap &mem, int hub, int spoke) {
			auto s = mem.vertex(spoke);
			for (auto h = mem.vertex(hub); h.first != h.second; ++h.first) {
				int i = *h.first;
				if (i == hub || i == spoke || ps.find(i) == ps.end()) {
					continue;
				}
				for (auto j = s.first; j != s.second; ++j) {
					if (*j == i) {
						return true;
					}
				}
			}
			return false;
		}

		void findTriangleIdx(const MeshTriangleMap &mtm, int &hub, int &spoke, Vector3 &face) {
			for (auto i = mtm.vertex(hub); i.first != i.second; ++i.first) {
				int tri = *i.first;
				int h = -1, s = -1;
				for (int j = 0; j != 3; ++j) {
					int vert = to.triangles[tri + j];
					if (vert == hub) {
						h = j;
					}
					else if (vert == spoke) {
						s = j;
					}
					else {
						face = to.vertices[vert];
					}
				}
				if (h != -1 && s != -1) {
					hub = h;
					spoke = s;
					return;
				}
			}

		}

		void pickNeighbours(const MeshTriangleMap &mtm, const MeshEdgeMap &mem, int vert, int &a, int &b) {
			a = -1;
			b = -1;
			for (auto i = mem.vertex(vert); i.first != i.second; ++i.first) {
				int j = *i.first;
				if (ps.find(j) == ps.end()) {
					continue;
				}
				if (a == -1) {
					a = j;
					continue;
				}
				if (b == -1) {
					b = j;
					continue;
				}
				if (neighbourOfNeighbour(mem, vert, a)) {
					if (neighbourOfNeighbour(mem, vert, b)) {
						a = leastNeighbours(mtm, a, b);
						if (neighbourOfNeighbour(mem, vert, j)) {
							a = leastNeighbours(mtm, a, j);
							b = leastNeighbours(mtm, b, j);
							return;
						}
						b = j;
						return;
					}
					if (neighbourOfNeighbour(mem, vert, j)) {
						b = leastNeighbours(mtm, a, j);
						return;
					}
					a = j;
					return;
				}
				if (neighbourOfNeighbour(mem, vert, b)) {
					if (neighbourOfNeighbour(mem, vert, j)) {
						b = leastNeighbours(mtm, b, j);
						return;
					}
					b = j;
					return;
				}
			}
		}

		void createSides() {
			to.getPerimeterSet(ps);
			feet.reserve(ps.size());
			MeshTriangleMap mtm(to);
			MeshEdgeMap mem(to);
			for (int i : ps) {
				int left, right;
				Vector3 face;
				pickNeighbours(mtm, mem, i, left, right);
				int h = i, s = left;
				findTriangleIdx(mtm, h, s, face);
				if (h > s) {
					addTriangle(i, addFoot(left), addFoot(i), face);
				}
				else {
					addTriangle(i, addFoot(i), left, face);
				}
				h = i;
				s = right;
				findTriangleIdx(mtm, h, s, face);
				if (h > s) {
					addTriangle(i, addFoot(right), addFoot(i), face);
				}
				else {
					addTriangle(i, addFoot(i), right, face);
				}
			}
		}

		void followPerimeter(int vert, std::vector<bool> set, const MeshTriangleMap &mtm, const MeshEdgeMap &mem) {
			int left, right, idx = vert;
			bool indent = true;
			for (float uv = 0.0f;; uv += 0.5f) {
				if (set[idx]) {
					return;
				}
				set[idx] = true;
				to.uv[idx].x = uv;
				if (indent) {
					to.vertices[idx] += to.normals[idx] * pertDist;
				}
				pickNeighbours(mtm, mem, idx, left, right);
				idx = set[left] ? right : left;
				indent = !indent;
			}
		}

		int findClosestOnPerimeter(int idx, const MeshEdgeMap &mem) {
			float dist = std::numeric_limits<float>::max();
			int out = -1;
			const Vector3 &v3 = to.vertices[idx];
			for (auto i = mem.vertex(idx); i.first != i.second; ++i.first) {
				int cmp = *i.first;
				if (ps.find(cmp) == ps.end()) {
					continue;
				}
				float d = (v3 - to.vertices[cmp]).squareMagnitude();
				if (d < dist) {
					out = cmp;
					dist = d;
				}
			}
			return out;
		}

		void setTopUVs(const MeshEdgeMap &mem, std::vector<bool> &set) {
			for (size_t i = 0, j = to.uv.size(); i != j; ++i) {
				if (ps.find(i) != ps.end()) {
					continue;
				}
				int idx = findClosestOnPerimeter(i, mem);
				if (idx == -1) {
					continue;
				}
				set[i] = true;
				auto uv = Vector2(to.uv[idx].x, 1.0f);
				to.uv[i] = uv;
				if (uv.x - static_cast<int>(uv.x) == 0.5f) {
					to.vertices[i] -= to.normals[i] * pertDist;
				}
			}
		}

		void setAverageUv(int idx, const MeshEdgeMap &mem, std::vector<bool> &set) {
			float totalX = 0.0f;
			int count = 0;
			for (auto i = mem.vertex(idx); i.first != i.second; ++i.first) {
				int off = *i.first;
				if (!set[off]) {
					continue;
				}
				totalX += to.uv[off].x;
				++count;
			}
			if (count == 0) {
				return;
			}
			set[idx] = true;
			to.uv[idx] = Vector2(totalX / count, 0.9f);
			to.vertices[idx].z += pertDist;
			for (auto i = mem.vertex(idx); i.first != i.second; ++i.first) {
				int off = *i.first;
				if (!set[off]) {
					setAverageUv(off, mem, set);
				}
			}
		}

		void setOtherUvs(const MeshEdgeMap &mem, std::vector<bool> &set) {
			for (size_t i = 0, j = to.uv.size(); i != j; ++i) {
				if (set[i]) {
					continue;
				}
				setAverageUv(i, mem, set);
			}
		}

		void setUVs() {
			size_t size = to.vertices.size();
			to.uv.resize(size, Vector2::zero());
			std::vector<bool> set(size, false);
			MeshEdgeMap mem(to);
			MeshTriangleMap mtm(to);
			to.calculateNormals(mtm);
			ps.clear();
			to.getPerimeterSet(ps);
			int left, right;
			for (int i : ps) {
				if (set[i]) {
					continue;
				}
				followPerimeter(i, set, mtm, mem);
			}
			setTopUVs(mem, set);
			setOtherUvs(mem, set);
		}
	};
}

MeshWithUV &Decoration::createForestMesh(const Mesh &in, MeshWithUV &out, float treeHeight) {
	MeshCopier copier(mesh, out, treeHeight);
	std::unordered_set<int> added(mesh.triangles.size() / 3);
	MeshTriangleMap tm(mesh);
	for (int i = 0, j = static_cast<int>(mesh.vertices.size()); i != j; ++i) {
		if (forest[i] != 0.0f) {
			for (auto j = tm.vertex(i); j.first != j.second; ++j.first) {
				int tri = *j.first;
				if (added.find(tri) != added.end()) {
					continue;
				}
				for (int k = 0;;) {
					if (forest[mesh.triangles[tri + k]] != 0.0f) {
						if (++k == 3) {
							added.insert(tri);
							copier.addTriangle(tri);
							break;
						}
					}
					else {
						break;
					}
				}
			}
		}
	}
	copier.avertLoops();
	out.tesselate();
	copier.createSides();
	copier.setUVs();
	return out;
}

std::vector<Vector3> &Decoration::getTrees(std::vector<Vector3> &out) const{
	return copyOutVertices(mesh, trees, out);
}

std::vector<Vector3> &Decoration::getBushes(std::vector<Vector3> &out) const{
	return copyOutVertices(mesh, bushes, out);
}

std::vector<Vector3> &Decoration::getRocks(std::vector<Vector3> &out) const{
	return copyOutVertices(mesh, rocks, out);
}

/*void Decoration::addRocks(const MeshEdgeMap &mep, std::default_random_engine &rnd) {
	std::uniform_real<float> full(0.0f, 1.0f);
	std::uniform_real<float> half(0.5f, 1.0f);
	std::uniform_real<float> quarter(0.25f, 0.75f);
	std::unordered_set<int> rocked;
	for (int i = 0, j = static_cast<int>(mesh.vertices.size()); i != j; ++i) {
		if (isLocalMaximum(mesh, mep, i)) {
			const Vector3 &pos = mesh.vertices[i];
			if (full(rnd) < getRockChance(mesh, mep, i)) {
				bigRocks.push_back(pos);
			}
			else {
				mediumRocks.push_back(pos);
			}
			rocked.insert(i);
		}
	}
	for (int i : rocked) {
		const Vector3 &pos = mesh.vertices[i];
		auto neighbours = mep.vertex(i);
		while (neighbours.first != neighbours.second) {
			int offset = *neighbours.first++;
			if (rocked.find(offset) != rocked.end()) {
				continue;
			}
			Vector3 dir = mesh.vertices[offset] - pos;
			if (full(rnd) < (1.0f - Vector3::unitZ().dot(mesh.normals[offset]))) {
				mediumRocks.push_back(pos + dir * half(rnd));
			}
			else {
				smallRocks.push_back(pos + dir * quarter(rnd));
			}
		}
	}
}*/

/*void Decoration::computeZValues(const Mesh &mesh) {
	ZAxisCollider collider(mesh);
	setZAxis(collider, bigRocks);
	setZAxis(collider, smallRocks);
	setZAxis(collider, mediumRocks);
	//setZAxis(collider, trees);
	treePositions.resize(trees.size());
	for (size_t i = 0, j = trees.size(); i != j; ++i) {
		treePositions[i] = mesh.vertices[trees[i]];
	}
	setZAxis(collider, bushes);
	setZAxis(collider, forestScatter);
}*/

std::ostream& motu::operator<<(std::ostream &out, const Decoration &decoration) {
	/*writeOutVector(out, decoration.treePositions);
	writeOutVector(out, decoration.bushes);
	writeOutVector(out, decoration.bigRocks);
	writeOutVector(out, decoration.mediumRocks);
	writeOutVector(out, decoration.smallRocks);
	writeOutVector(out, decoration.forestScatter);*/
	writeOutVector(out, decoration.forest);
	writeOutVector(out, decoration.bushes);
	writeOutVector(out, decoration.rocks);
	out << decoration.mesh;
	writeOutVector(out, decoration.forest);
	writeOutVector(out, decoration.soilRichness);
	return out;
}

std::istream& motu::operator>>(std::istream &in, Decoration &decoration) {
	/*readInVector(in, decoration.treePositions);
	readInVector(in, decoration.bushes);
	readInVector(in, decoration.bigRocks);
	readInVector(in, decoration.mediumRocks);
	readInVector(in, decoration.smallRocks);
	readInVector(in, decoration.forestScatter);*/
	readInVector(in, decoration.forest);
	readInVector(in, decoration.bushes);
	readInVector(in, decoration.rocks);
	in >> decoration.mesh;
	readInVector(in, decoration.forest);
	readInVector(in, decoration.soilRichness);
	return in;
}