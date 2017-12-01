#include "triangulated_terrain_graph.h"
#include "terrain_graph.h"
#include "raster.h"
#include "brtree.h"

using namespace motu;

namespace {
	typedef std::map<Vector2, const TerrainGraph::Vertex*> VertexMap;
	typedef BRTree<TriangulatedTerrainGraph::Triangulation::Face*> FaceTree;

	typedef std::vector<TriangulatedTerrainGraph::Triangulation::Vertex*> SeaErosianList;

	void seaErode(TriangulatedTerrainGraph::Triangulation::Vertex &vert, SeaErosianList list) {
		int seaCount = 0, landCount = 0;
		for (auto i = vert.inbound().begin(); i != vert.inbound().end(); ++i) {
			if (i->next->vertex().data().z < 0.0f) {
				++seaCount;
			}
			else {
				++landCount;
			}
		}
		if (vert.data().z < 0.0f) {
			if (landCount > seaCount) {
				list.push_back(&vert);
			}
		}
		else if (seaCount > landCount) {
			list.push_back(&vert);
		}
	}

	template<class VertexMap>
	void interpolateZValue(const VertexMap &vertexMap, TriangulatedTerrainGraph::Triangulation::Vertex &vert) {
		float z = 0.0f;
		int count = 0;
		for (auto i = vert.inbound().begin(); i != vert.inbound().end(); ++i) {
			auto j = vertexMap.find(i->next->vertex().position());
			if (j != vertexMap.end()) {
				z += j->second->data().z;
				++count;
			}
		}
		vert.data().z = z / count;
	}

	template <class VertexMap>
	void setZValues(const VertexMap &vertexMap, TriangulatedTerrainGraph::Triangulation &tri) {
		for (auto i = tri.vertices().begin(); i != tri.vertices().end(); ++i) {
			auto j = vertexMap.find(i->position());
			if (j != vertexMap.end()) {
				i->data().z = j->second->data().z;
			}
			else {
				interpolateZValue(vertexMap, *i);
			}
		}
		for (auto i = tri.vertices().begin(); i != tri.vertices().end(); ++i) {
			if (i->data().z < -0.002f) {
				i->data().z = -0.002f;
			}
		}
	}

	void trackErosian(TriangulatedTerrainGraph::Triangulation::Face &face, float carryCapacity) {
		typedef TriangulatedTerrainGraph::Triangulation::Vertex Vertex;
		float speed = 0.0f, carrying = 0.0f, lowest = std::numeric_limits<float>::max();
		Vertex *next = nullptr;
		for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i) {
			if (i->vertex().data().z < lowest) {
				lowest = i->vertex().data().z;
				next = &i->vertex();
			}
		}
		if (!next || lowest <= 0.0f) {
			return;
		}
		while (true) {
			Vertex *down = nullptr;
			for (auto i = next->inbound().begin(); i != next->inbound().end(); ++i) {
				if (i->next->vertex().data().z < lowest) {
					down = i->next->mVertex;
					lowest = down->data().z;
				}
			}
			if (lowest < 0.0f) {
				return;
			}
			if (!down) {
				next->data().z += carrying;
				return;
			}
			speed += next->data().z - lowest;
			speed *= 0.5f;
			float capacity = speed * carryCapacity;
			down->data().z += (capacity - carrying);
			carrying = capacity;
			next = down;
		}
	}

	bool isSea(const TriangulatedTerrainGraph::Triangulation::Vertex &vert) {
		for (auto i = vert.inbound().begin(); i != vert.inbound().end(); ++i) {
			if (i->next->vertex().data().z >= 0.0f) {
				return false;
			}
		}
		return true;
	}
	
	void findFlow(TriangulatedTerrainGraph &tg) {
		for (auto i = tg.triangulation().vertices().begin(); i != tg.triangulation().vertices().end(); ++i) {
			i->data().flow = 0;
			if (i->data().z < 0.0f && isSea(*i)) {
				continue;
			}
			float lowest = std::numeric_limits<float>::max();
			Vector3 pos(i->position().x, i->position().y, i->data().z);
			for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j) {
				if (&j->next->vertex() == &*i) {
					continue;
				}
				float z = j->next->vertex().data().z;
				if (z < lowest) {
					i->data().down = &j->next->vertex();
					lowest = z;
				}
			}
			if (lowest > i->data().z) {
				i->data().down->data().z = i->data().z;
			}
		}
		typedef TriangulatedTerrainGraph::Triangulation::Vertex Vertex;
		std::set<const Vertex *> visited;
		for (auto i = tg.triangulation().vertices().begin(); i != tg.triangulation().vertices().end(); ++i) {
			visited.clear();
			for (Vertex *v = i->data().down; v; v = v->data().down) {
				if (v->data().down && visited.find(v->data().down) != visited.end()) {
					float lowest = std::numeric_limits<float>::max();
					for (auto j = v->inbound().begin(); j != v->inbound().end(); ++j) {
						if (visited.find(&j->pair->vertex()) != visited.end()) {
							continue;
						}
						float z = j->next->vertex().data().z;
						if (z < lowest) {
							v->data().down = &j->next->vertex();
							lowest = z;
						}
					}
					if (lowest > v->data().z) {
						v->data().down->data().z = v->data().z;
					}
				}
				++v->data().flow;
				visited.insert(v);
			}
		}
	}

	struct NotFlat {
		bool operator()(const TriangulatedTerrainGraph::Triangulation::Face &face) const{
			float z = face.halfEdge()->vertex().data().z;
			for (auto i = face.halfEdge()->next; i != face.halfEdge(); i = i->next) {
				if (i->vertex().data().z != z) {
					return true;
				}
			}
			return false;
		}
	};

	struct All {
		bool operator()(const TerrainGraph::Face &face) const {
			return true;
		}
	};

	template <class Graph, class VertexMap, class AddCentroid>
	std::vector<Vector2> copyVertices(VertexMap &vertexMap, Graph &graph, std::vector<Vector2> &vertices, AddCentroid addCentroid = All()) {
		vertices.reserve(graph.vertices().size() + graph.faces().size());
		for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i) {
			vertexMap.emplace(i->position(), &*i);
			vertices.push_back(i->position());
		}
		for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i) {
			if (&*i == &graph.externalFace() || !addCentroid(*i)) {
				continue;
			}
			vertices.push_back(i->calculateCentroid());
		}
		return vertices;
	}
}

TriangulatedTerrainGraph::TriangulatedTerrainGraph(const TerrainGraph &graph) {
	std::map<Vector2, const TerrainGraph::Vertex*> vertexMap;
	std::vector<Vector2> vertices;
	mTriangulation = new Triangulation(copyVertices(vertexMap, graph, vertices, All()));
	setZValues(vertexMap, *mTriangulation);
}

TriangulatedTerrainGraph::TriangulatedTerrainGraph(const TriangulatedTerrainGraph &graph) {
	std::map<Vector2, const Triangulation::Vertex*> vertexMap;
	std::vector<Vector2> vertices;
	mTriangulation = new Triangulation(copyVertices(vertexMap, *graph.mTriangulation, vertices, NotFlat()));
	setZValues(vertexMap, *mTriangulation);
}

void TriangulatedTerrainGraph::toMesh(Mesh &mesh) const {
	mesh.vertices.reserve(mTriangulation->vertices().size());
	mesh.triangles.reserve((mTriangulation->faces().size() - 1) * 3);
	mesh.normals.reserve(mTriangulation->vertices().size());
	std::map<const Triangulation::Vertex*, size_t> indices;
	for (auto i = mTriangulation->faces().begin(); i != mTriangulation->faces().end(); ++i) {
		if (&*i == &mTriangulation->externalFace()) {
			continue;
		}
		const Triangulation::HalfEdgeType *h = i->halfEdge();
		do {
			const Triangulation::Vertex *vert = &h->vertex();
			auto j = indices.find(vert);
			if (j == indices.end()) {
				j = indices.emplace(vert, mesh.vertices.size()).first;
				mesh.vertices.emplace_back(vert->position().x, vert->position().y, vert->data().z);
			}
			mesh.triangles.push_back(j->second);
			h = h->next;
		} while (h != i->halfEdge());
	}
}

void TriangulatedTerrainGraph::erode(int drops, float carryCapacity) {
	while (drops--) {
		for (auto i = mTriangulation->faces().begin(); i != mTriangulation->faces().end(); ++i) {
			if (&*i == &mTriangulation->externalFace()) {
				continue;
			}
			trackErosian(*i, carryCapacity);
		}
	}
	SeaErosianList list;
	list.reserve(mTriangulation->vertices().size());
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		seaErode(*i, list);
	}
	for (auto i = list.begin(); i != list.end(); ++i) {
		(*i)->data().z = 0.0f;
	}
}

void TriangulatedTerrainGraph::copyBackZValues(const Grid<Vector3> &grid) {
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		int x = i->position().x * grid.width();
		int y = i->position().y * grid.height();
		if (x < 0.0f || y < 0.0f) {
			continue;
		}
		if (x >= grid.width()) {
			x = grid.width() - 1;
		}
		if (y >= grid.height()) {
			y = grid.height() - 1;
		}
		i->data().z = grid(x, y).z;
	}
}

Rivers::Edges &TriangulatedTerrainGraph::findRivers(Rivers::Edges &edges, float thresholdStandardDeviations) {
	findFlow(*this);
	int total = 0, count = 0;
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		int flow = i->data().flow;
		if (flow) {
			total += flow;
			++count;
		}
	}
	float mean = static_cast<float>(total) / count, variance = 0.0;
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		int flow = i->data().flow;
		if (flow) {
			float v = (flow - mean);
			variance += v * v;
		}
	}
	variance /= total;
	float target = sqrtf(variance) * thresholdStandardDeviations;
	std::vector<std::pair<Triangulation::Vertex *, const Triangulation::Vertex *>> lastList;
	lastList.reserve(mTriangulation->vertices().size());
	float maxZ = 0.0f, maxFlow = 0.0f;
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		if (i->data().down && (i->data().flow - mean) >= target) {
			if (i->data().z > maxZ) {
				maxZ = i->data().z;
			}
			if (i->data().flow > maxFlow) {
				maxFlow = i->data().flow;
			}
			lastList.emplace_back(i->data().down, &*i);
			edges.emplace_back(i->position(), i->data().down->position());
		}
	}
	float flowMul = maxZ / maxFlow;
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		i->first->data().z -= sqrtf(i->first->data().flow) * flowMul;
	}
	std::vector<std::pair<Triangulation::Vertex *, Vector3>> adjusted;
	adjusted.reserve(lastList.size());
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		Vector3 last(i->second->position().x, i->second->position().y, i->second->data().z);
		Vector3 current(i->first->position().x, i->first->position().y, i->first->data().z);
		if (i->first->data().down) {
			Vector3 next(i->first->data().down->position().x, i->first->data().down->position().y, i->first->data().down->data().z);
			adjusted.emplace_back(i->first, (last + current + next) / 3.0f);
		}
		else {
			adjusted.emplace_back(i->first, (last + current) * 0.5f);
		}
	}
	for (auto i = adjusted.begin(); i != adjusted.end(); ++i) {
		i->first->position().x = i->second.x;
		i->first->position().y = i->second.y;
		i->first->data().z = i->second.z;
	}
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		edges.emplace_back(i->first->position(), i->second->position());
	}
	return edges;
}

void TriangulatedTerrainGraph::smooth() {
	std::vector<std::pair<Triangulation::Vertex*, Vector3>> adjusted;
	adjusted.reserve(mTriangulation->vertices().size());
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		Vector3 total(i->position().x, i->position().y, i->data().z);
		int count = 1;
		for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j) {
			const Vector2 &pos = j->pair->vertex().position();
			total.x += pos.x;
			total.y += pos.y;
			total.z += j->pair->vertex().data().z;
			++count;
		}
		adjusted.emplace_back(&*i, total / count);
	}
	for (auto i = adjusted.begin(); i != adjusted.end(); ++i) {
		i->first->position().x = i->second.x;
		i->first->position().y = i->second.y;
		i->first->data().z = i->second.z;
	}
}