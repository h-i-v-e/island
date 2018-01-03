#include <map>
#include <set>
#include <cmath>

#include "mesh.h"
#include "mesh_decimator.h"
#include "object_pool.h"
#include <memory>

using namespace motu;

namespace {
	struct MeshVertex {
		Vector3 position;
		std::vector<uint32_t> triangles;
		//bool edge;

		MeshVertex(const Vector3 &pos) : position(pos) {}

		/*void setEdge(const Mesh &mesh, std::map<uint32_t, uint32_t> &counters) {
			counters.clear();
			for (uint32_t tri : triangles) {
				for (uint32_t i = 0; i != 3; ++i) {
					uint32_t offset = mesh.triangles[tri + i];
					auto j = counters.find(offset);
					if (j == counters.end()) {
						counters.emplace_hint(j, offset, 1);
					}
					else {
						++j->second;
					}
				}
			}
			for (auto i = counters.begin(); i != counters.end(); ++i) {
				if (i->second == 1) {
					edge = true;
					return;
				}
			}
			edge = false;
		}*/
	};

	struct CollapsePoint {
		uint32_t vertA, vertB;
		Vector3 collapseTo;
		float cost;
		uint32_t triangles;

		CollapsePoint() {}

		CollapsePoint(uint32_t vertA, uint32_t vertB) : vertA(vertA), vertB(vertB) {
		}

		typedef std::shared_ptr<CollapsePoint> Ptr;
	};

	//typedef ObjectPool<CollapsePoint> CollapsePointPool;

	struct CollapsePointCompare {
		bool operator()(const CollapsePoint::Ptr &a, const CollapsePoint::Ptr &b) const {
			if (a->cost < b->cost) {
				return true;
			}
			if (a->cost > b->cost) {
				return false;
			}
			if (a->triangles < b->triangles) {
				return true;
			}
			if (a->triangles > b->triangles) {
				return false;
			}
			if (a->vertA < b->vertA) {
				return true;
			}
			if (a->vertA > b->vertA) {
				return false;
			}
			return a->vertB < b->vertB;
		}
	};

	struct VertexPair {
		uint32_t first, second;

		VertexPair(uint32_t a, uint32_t b) {
			if (a < b) {
				first = a;
				second = b;
			}
			else {
				first = b;
				second = a;
			}
		}

		bool operator < (const VertexPair &other) const {
			if (first < other.first) {
				return true;
			}
			if (first > other.first) {
				return false;
			}
			return second < other.second;
		}
	};

	struct MeshMap {
		typedef std::set<VertexPair> VertexPairs;
		typedef std::set<CollapsePoint::Ptr, CollapsePointCompare> OrderedPairs;
		typedef std::multimap<size_t, CollapsePoint::Ptr> VertexMap;

		OrderedPairs waiting;
		VertexMap vertMap;
		VertexPairs added;
		std::vector<MeshVertex> vertices;
		Mesh *mesh;
		float minAngle;

		MeshMap(Mesh &mesh, float minAngle) : mesh(&mesh), minAngle(cosf(minAngle * acos(-1) / 180.0)) {
			vertices.reserve(mesh.vertices.size());
			for (const Vector3 &vec : mesh.vertices) {
				vertices.emplace_back(vec);
			}
			for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
				for (size_t j = -2; j != 1; ++j) {
					size_t offset = mesh.triangles[i + j];
					MeshVertex &vert = vertices[offset];
					if (std::find(vert.triangles.begin(), vert.triangles.end(), i - 2) == vert.triangles.end()) {
						vert.triangles.push_back(i - 2);
					}
				}
			}
			/*std::map<uint32_t, uint32_t> counters;
			for (MeshVertex &vert : vertices) {
				vert.setEdge(mesh, counters);
			}*/
			for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
				addCollapsePoints(i - 2);
			}
		}

		void copyBack() {
			std::set<uint32_t> usedTriangles;
			std::map<uint32_t, uint32_t> vertexMap;
			std::vector<Vector3> newVertices;
			newVertices.reserve(mesh->vertices.size());
			for (uint32_t i = 0; i != vertices.size(); ++i) {
				if (!vertices[i].triangles.empty()) {
					vertexMap.emplace(i, newVertices.size());
					newVertices.push_back(vertices[i].position);
					for (uint32_t tri : vertices[i].triangles) {
						usedTriangles.insert(tri);
					}
				}
			}
			mesh->vertices.resize(0);
			std::copy(newVertices.begin(), newVertices.end(), std::back_inserter(mesh->vertices));
			std::vector<uint32_t> newTriangles;
			newTriangles.reserve(mesh->triangles.size());
			for (uint32_t tri : usedTriangles) {
				newTriangles.push_back(vertexMap[mesh->triangles[tri]]);
				newTriangles.push_back(vertexMap[mesh->triangles[tri + 1]]);
				newTriangles.push_back(vertexMap[mesh->triangles[tri + 2]]);
			}
			mesh->triangles.resize(0);
			std::copy(newTriangles.begin(), newTriangles.end(), std::back_inserter(mesh->triangles));
		}

		void addCollapsePoint(const VertexPair &vp) {
			if (added.find(vp) == added.end()/* && !(vertices[vp.first].edge || vertices[vp.second].edge)*/) {
				added.insert(vp);
				CollapsePoint::Ptr cp = std::make_shared<CollapsePoint>(vp.first, vp.second);
				findCollapsePoint(*cp);
				vertMap.emplace(vp.first, cp);
				vertMap.emplace(vp.second, cp);
				waiting.insert(cp);
			}
		}

		constexpr bool validAngle(float cos) const{
			return cos <= minAngle && cos >= -minAngle;
		}

		bool validTriangle(uint32_t t1, uint32_t t2, uint32_t t3) const{
			const Vector3 &a = vertices[t1].position, &b = vertices[t2].position, &c = vertices[t3].position;
			Vector3 ab((b - a).normalize()), bc((b - c).normalize());
			if (validAngle(ab.dot(bc))) {
				return false;
			}
			Vector3 ca((a - c).normalize());
			return validAngle(bc.dot(ca)) && validAngle(ca.dot(ab));
		}

		void addCollapsePoints(uint32_t triangle) {
			uint32_t a = mesh->triangles[triangle], b = mesh->triangles[triangle + 1], c = mesh->triangles[triangle + 2];
			//if (validTriangle(a, b, c)) {
				addCollapsePoint(VertexPair(a, b));
				addCollapsePoint(VertexPair(b, c));
				addCollapsePoint(VertexPair(c, a));
			//}
		}

		void replace(uint32_t old, uint32_t nw) {
			MeshVertex &vertA = vertices[nw], &vertB = vertices[old];
			if (vertA.triangles.empty() || vertB.triangles.empty()) {
				std::cout << "Oh dear" << std::endl;
			}
			std::vector<uint32_t> remove;
			remove.reserve(vertA.triangles.size() + vertB.triangles.size());
			for (uint32_t tri : vertA.triangles) {
				for (uint32_t i = 0; i != 3; ++i) {
					if (mesh->triangles[tri + i] == old) {
						remove.push_back(tri);
						break;
					}
				}
			}
			for (uint32_t tri : vertB.triangles) {
				uint32_t op;
				bool set = true;
				for (uint32_t i = 0; i != 3; ++i) {
					uint32_t j = mesh->triangles[tri + i];
					if (j == old) {
						op = tri + i;
					}
					else if (j == nw) {
						remove.push_back(tri);
						set = false;
						break;
					}
				}
				if (set) {
					mesh->triangles[op] = nw;
					vertA.triangles.push_back(tri);
				}
			}
			vertB.triangles.resize(0);
			vertB.triangles.shrink_to_fit();
			for (uint32_t tri : remove) {
				for (int i = 0; i != 3; ++i) {
					auto &triangles = vertices[mesh->triangles[tri + i]].triangles;
					auto j = std::find(triangles.begin(), triangles.end(), tri);
					if (j != triangles.end()) {
						triangles.erase(j);
					}
				}
			}
		}

		float distanceToPlanesAround(uint32_t offset, const Vector3 &point) {
			const MeshVertex &vert = vertices[offset];
			float total = 0.0f;
			for (uint32_t i : vert.triangles) {
				Triangle3 t3(vertices[mesh->triangles[i]].position, vertices[mesh->triangles[i + 1]].position, vertices[mesh->triangles[i + 2]].position);
				float distance = Plane(t3.vertices[0], t3.normal().normalized()).distanceTo(point);
				if (distance < 0.0f) {
					distance = -distance;
				}
				total += distance;
			}
			return total;
		}

		void findCollapsePoint(CollapsePoint &target) {
			const Vector3 vertA = vertices[target.vertA].position, vertB = vertices[target.vertB].position;
			float a = distanceToPlanesAround(target.vertB, vertA);
			float b = distanceToPlanesAround(target.vertA, vertB);
			Vector3 mid(vertA + ((vertB - vertA) * 0.5f));
			float c = distanceToPlanesAround(target.vertB, mid) + distanceToPlanesAround(target.vertA, mid);
			if (a < b) {
				if (a < c) {
					target.collapseTo = vertA;
					target.cost = a;
					return;
				}
			}
			else if (b < c) {
				target.collapseTo = vertB;
				target.cost = b;
				return;
			}
			target.collapseTo = mid;
			target.cost = c;
			target.triangles = vertices[target.vertA].triangles.size() + vertices[target.vertB].triangles.size();
		}

		int decimate(int targetVertices, bool decimateAllFree) {
			std::set<CollapsePoint::Ptr> update;
			int num = mesh->vertices.size() - targetVertices;
			int count = 0;
			while (!waiting.empty()) {
				update.clear();
				CollapsePoint::Ptr cp = *waiting.begin();
				--num;
				if ((!(decimateAllFree && cp->cost == 0.0f)) && num < 0) {
					copyBack();
					return count;
				}
				++count;
				waiting.erase(waiting.begin());
				auto j = vertMap.equal_range(cp->vertA);
				for (auto k = j.first; k != j.second; ++k) {
					if (k->second != cp) {
						update.insert(k->second);
					}
				}
				vertMap.erase(j.first, j.second);
				j = vertMap.equal_range(cp->vertB);
				for (auto k = j.first; k != j.second; ++k) {
					if (k->second != cp) {
						update.insert(k->second);
					}
				}
				vertMap.erase(j.first, j.second);
				for (auto k = update.begin(); k != update.end(); ++k) {
					waiting.erase(*k);
					added.erase(VertexPair((*k)->vertA, (*k)->vertB));
				}
				vertices[cp->vertA].position = cp->collapseTo;
				replace(cp->vertB, cp->vertA);
				for (uint32_t tri : vertices[cp->vertA].triangles) {
					addCollapsePoints(tri);
				}
			}
			copyBack();
			return count;
		}
	};
}

int motu::decimate(Mesh &mesh, int targetVertices, bool decimateAllFree, float minAngle) {
	if (mesh.manifold()) {
		std::cout << "yay" << std::endl;
	}
	return MeshMap(mesh, minAngle).decimate(targetVertices, decimateAllFree);
}

