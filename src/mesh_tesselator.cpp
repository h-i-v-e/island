#include <unordered_map>
#include <iostream>

#include "mesh_tesselator.h"
#include "mesh.h"
#include "util.h"

using namespace motu;

namespace {
	struct VertexPair {
		int a, b;

		VertexPair(int a, int b) {
			if (a < b) {
				this->a = a;
				this->b = b;
			}
			else {
				this->a = b;
				this->b = a;
			}
		}

		constexpr size_t hash() const{
			return (a * HASH_PRIME_A) ^ (b * HASH_PRIME_B);
		}

		constexpr bool operator == (const VertexPair &other) const {
			return a == other.a && b == other.b;
		}

		constexpr bool operator != (const VertexPair &other) const {
			return a != other.a || b != other.b;
		}
	};

	struct VertexCache {
		Mesh &output;
		std::unordered_map<VertexPair, int, Hasher<VertexPair>> cache;

		VertexCache(Mesh &output) : output(output){
			size_t size = output.vertices.size() << 2;
			output.vertices.reserve(size);
			output.triangles.reserve((size - 2) * 3);
			cache.reserve(size);
		}

		int createCentre(int a, int b) {
			const Vector3 &v3a = output.vertices[a], v3b = output.vertices[b];
			int out = output.vertices.size();
			output.vertices.push_back((v3a + v3b) * 0.5f);
			return out;
		}

		int get(int a, int b) {
			VertexPair pair(a, b);
			auto i = cache.find(pair);
			if (i == cache.end()) {
				int output = createCentre(a, b);
				cache.emplace(pair, output);
				return output;
			}
			return i->second;
		}

		int getNoSplit(int a, int b) {
			VertexPair pair(a, b);
			auto i = cache.find(pair);
			return i == cache.end() ? -1 : i->second;
		}

		Vector3 getUp(int a, int b, int c) {
			const Vector3 &va = output.vertices[a];
			return (output.vertices[b] - va).cross(output.vertices[c] - va);
		}

		void addTriangle(int a, int b, int c) {
			output.triangles.push_back(a);
			output.triangles.push_back(b);
			output.triangles.push_back(c);
		}

		void addTriangle(int a, int b, int c, const Vector3 &up, int step) {
			output.triangles.push_back(a);
			output.triangles.push_back(b);
			output.triangles.push_back(c);
		}

		void addTriangles(int a, int b, int c, int ab, int ac, int cb) {
			addTriangle(a, ab, ac);
			addTriangle(b, cb, ab);
			addTriangle(c, ac, cb);
			addTriangle(ab, cb, ac);
		}

		void tesselate(int a, int b, int c) {
			addTriangles(a, b, c, get(a, b), get(a, c), get(c, b));
		}

		void addNoSplit(int a, int b, int c) {
			Vector3 up = getUp(a, b, c);
			int ab = getNoSplit(a, b);
			int ac = getNoSplit(a, c);
			int cb = getNoSplit(c, b);
			if (ab != -1) {//0
				if (ac != -1) {//1
					if (cb != -1) {
						addTriangles(a, b, c, ab, ac, cb);
						return;
					}
					addTriangle(ab, ac, a, up, 0);
					addTriangle(c, ac, b, up, 1);
					addTriangle(b, ac, ab, up, 2);
					return;
				}
				addTriangle(a, ab, c, up, 3);
				addTriangle(b, c, ab, up, 4);
				return;
			}
			if (ac != -1) {//2
				if (cb != -1) {//3
					addTriangle(a, cb, ac, up, 5);
					addTriangle(ac, cb, c, up, 6);
					addTriangle(b, cb, a, up, 7);
					return;
				}
				addTriangle(a, b, ac, up, 8);
				addTriangle(b, c, ac, up, 9);
				return;
			}
			if (cb != -1) {//4
				addTriangle(a, cb, c, up, 10);
				addTriangle(a, b, cb, up, 11);
				return;
			}
			addTriangle(a, b, c);
		}
	};
}

Mesh &motu::tesselate(Mesh &to) {
	//std::cout << "Before: " << to.vertices.size() << std::endl;
	if (to.vertices.size() < 3) {
		return to;
	}
	VertexCache cache(to);
	std::vector<int> triangleBuffer(to.triangles);
	to.triangles.clear();
	for (size_t i = 0, j = triangleBuffer.size(); i != j; i += 3) {
		cache.tesselate(triangleBuffer[i], triangleBuffer[i + 1], triangleBuffer[i + 2]);
	}
	//std::cout << "After: " << to.vertices.size() << std::endl;
	return to;
}

MeshWithUV &motu::tesselate(MeshWithUV &mesh) {
	tesselate(*reinterpret_cast<Mesh*>(&mesh));
	if (mesh.uv.empty()) {
		return mesh;
	}
	size_t old = mesh.uv.size(), nw = mesh.vertices.size();
	mesh.uv.resize(nw);
	MeshEdgeMap mem(mesh);
	for (size_t i = 0; i != nw; ++i) {
		Vector2 total = Vector2::zero();
		int count = 0;
		for (auto j = mem.vertex(old); j.first != j.second; ++j.first) {
			int idx = *j.first;
			if (idx < old) {
				total += mesh.uv[idx];
				if (++count == 2) {
					break;
				}
			}
		}
		mesh.uv[i] = total * 0.5f;
	}
	return mesh;
}

Mesh &motu::tesselate(Mesh &to, float greaterThan) {
	//std::cout << "Before: " << to.vertices.size() << std::endl;
	VertexCache cache(to);
	std::vector<int> triangleBuffer(to.triangles);
	to.triangles.clear();
	std::vector<int> skip;
	skip.reserve(to.triangles.size() / 3);
	for (size_t i = 0, j = triangleBuffer.size(); i != j; i += 3) {
		int a = triangleBuffer[i], b = triangleBuffer[i + 1], c = triangleBuffer[i + 2];
		if (to.vertices[a].z > greaterThan || to.vertices[b].z > greaterThan || to.vertices[c].z > greaterThan) {
			cache.tesselate(a, b, c);
		}
		else {
			skip.push_back(i);
		}
	}
	for (int i : skip) {
		cache.addNoSplit(triangleBuffer[i], triangleBuffer[i + 1], triangleBuffer[i + 2]);
	}
	return to;
}

Mesh &motu::tesselate(Mesh &mesh, std::unordered_set<int> &in) {
	VertexCache cache(mesh);
	std::vector<int> triangleBuffer(mesh.triangles);
	mesh.triangles.clear();
	std::vector<int> skip;
	skip.reserve(mesh.triangles.size() / 3);
	for (size_t i = 0, j = triangleBuffer.size(); i != j; i += 3) {
		int a = triangleBuffer[i], b = triangleBuffer[i + 1], c = triangleBuffer[i + 2];
		if (in.find(a) != in.end()) {
			cache.tesselate(a, b, c);
		}
		else {
			skip.push_back(i);
		}
	}
	for (int i : skip) {
		cache.addNoSplit(triangleBuffer[i], triangleBuffer[i + 1], triangleBuffer[i + 2]);
	}
	return mesh;
}