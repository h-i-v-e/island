#include "triangulated_terrain_graph.h"
#include "terrain_graph.h"
#include "raster.h"
#include "brtree.h"
#include "plane.h"

using namespace motu;

namespace {
	typedef std::map<Vector2, const TerrainGraph::Vertex*> VertexMap;
	typedef BRTree<TriangulatedTerrainGraph::Triangulation::Face*> FaceTree;
	typedef TriangulatedTerrainGraph::Triangulation::Vertex Vertex;
	typedef TriangulatedTerrainGraph::Triangulation::Face Face;
	typedef std::vector<Vertex*> SeaErosianList;

	void seaErode(Vertex &vert, SeaErosianList list) {
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
	void interpolateZValue(const VertexMap &vertexMap, Vertex &vert) {
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

	void trackErosian(Face &face, float carryCapacity) {
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

	bool isSea(const Vertex &vert) {
		for (auto i = vert.inbound().begin(); i != vert.inbound().end(); ++i) {
			if (i->next->vertex().data().z >= 0.0f) {
				return false;
			}
		}
		return true;
	}

	Vector3 toVector3(const Vertex &vert) {
		return Vector3(vert.position().x, vert.position().y, vert.data().z);
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
				float z = (toVector3(j->next->vertex()) - pos).normalized().z;//j->next->vertex().data().z;
				if (z < lowest) {
					i->data().down = &j->next->vertex();
					lowest = z;
				}
			}
			if (lowest > i->data().z && i->data().down) {
				i->data().down->data().z = i->data().z - FLT_EPSILON;
			}
		}
		std::set<const Vertex *> visited;
		for (auto i = tg.triangulation().vertices().begin(); i != tg.triangulation().vertices().end(); ++i) {
			visited.clear();
			for (Vertex *v = i->data().down; v; v = v->data().down) {
				if (v->data().down && visited.find(v->data().down) != visited.end() || visited.find(v) != visited.end()) {
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
						v->data().down->data().z = v->data().z - FLT_EPSILON;
					}
				}
				++v->data().flow;
				visited.insert(v);
			}
		}
	}

	struct NotFlat {
		bool operator()(const Face &face) const {
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

	struct RiverSection {
		const Vertex *vertex;
		float waterDepth;

		float surfaceZ() const {
			return vertex->data().z + waterDepth;
		}

		RiverSection(const Vertex *vertex, float waterDepth) : vertex(vertex), waterDepth(waterDepth) {}
	};

	struct RiverSections : public ObjectPool<RiverSection>{
		RiverSection *last, *current, *next;

		RiverSections() : ObjectPool<RiverSection>(3) {
			last = current = next = nullptr;
		}

		void releaseIfNotNull(RiverSection *section) {
			if (section) {
				release(section);
			}
		}

		void clear() {
			releaseIfNotNull(last);
			last = nullptr;
			releaseIfNotNull(current);
			current = nullptr;
			releaseIfNotNull(next);
			next = nullptr;
		}

		void add(RiverSection *section) {
			if (last) {
				release(last);
			}
			last = current;
			current = next;
			next = section;
		}

		static Vector3 getPosition(const RiverSection &endA, const RiverSection &endB, const Vertex &vertex) {
			Edge flowLine(endA.vertex->position(), endB.vertex->position());
			Vector2 flowDirection(flowLine.direction());
			float intersectionTime;
			Edge(vertex.position(), vertex.position() + flowDirection.normal()).intersectionTime(flowLine, intersectionTime);
			float dz = endB.surfaceZ() - endA.surfaceZ();
			float surfaceZ = endA.surfaceZ() + (dz * intersectionTime);
			dz = endB.vertex->data().z - endA.vertex->data().z;
			float z = endA.vertex->data().z + (dz * intersectionTime);
			float depth = vertex.data().z - surfaceZ;
			float zDistance = vertex.data().z - z;
			if (zDistance < FLT_EPSILON) {
				return toVector3(vertex);
			}
			float delta = depth / zDistance;
			//std::cout << delta << std::endl;
			if (delta < FLT_EPSILON) {
				delta = FLT_EPSILON;
			}
			else if (delta > (1.0f - FLT_EPSILON)) {
				delta = 1.0f - FLT_EPSILON;
			}
			Vector2 intersectionPoint(endA.vertex->position() + (flowDirection * intersectionTime));
			Vector2 shifted(vertex.position() + ((intersectionPoint - vertex.position()) * delta));
			return Vector3(shifted.x, shifted.y, surfaceZ);
		}

		Vector3 getPosition(const Vertex *vertex) const{
			if (last) {
				if (next){
					for (auto i = next->vertex->inbound().begin(); i != next->vertex->inbound().end(); ++i) {
						if (i->next->mVertex == vertex) {
							return getPosition(*current, *next, *vertex);
						}
					}
				}
				return getPosition(*last, *current, *vertex);
			}
			return getPosition(*current, *next, *vertex);
		}

		bool validVertex(const Vertex *vertex) const{
			return !((last && last->vertex == vertex) || (next && next->vertex == vertex));
		}
	};

	void addTriangles(std::vector<Vector3> &points, std::vector<Triangle3> &triangles, const RiverSections &sections) {
		points.clear();
		Vector3 pos(toVector3(*sections.current->vertex));
		for (auto i = sections.current->vertex->inbound().begin(); i != sections.current->vertex->inbound().end(); ++i) {
			if (sections.validVertex(i->next->mVertex)) {
				points.push_back(sections.getPosition(i->next->mVertex));
			}
		}
		pos.z += sections.current->waterDepth;
		for (int i = 1; i < points.size(); ++i) {
			triangles.emplace_back(pos, points[i - 1], points[i]);
		}
		triangles.emplace_back(pos, points.back(), points.front());
	}

	void addTriangles(TriangulatedTerrainGraph::LastList &lastList, std::vector<Triangle3> &triangles) {
		std::set<const Vertex*> vertexSet, visitedSet;
		std::vector<Vector3> points;
		Plane splitPlane;
		Spline lastVertices;
		for (auto i = lastList.begin(); i != lastList.end(); ++i) {
			vertexSet.insert(i->first);
		}
		RiverSections sections;
		for (auto i = lastList.begin(); i != lastList.end(); ++i) {
			if (vertexSet.find(i->second.last) != vertexSet.end()) {
				continue;
			}
			points.clear();
			sections.add(sections.allocate(i->second.last, 0.0f));
			float currentDepth = i->second.last->data().z;
			const Vertex *last = i->second.last;
			bool wasBranch = false;
			for (const Vertex *vert = i->first; vert; vert = vert->data().down) {
				float depth = sqrtf(vert->data().flow) * lastList.flowMultiplier * 0.01f;
				depth = std::min(currentDepth - vert->data().z, depth);
				depth = std::max(0.0f, depth);
				currentDepth = vert->data().z + depth;
				sections.add(sections.allocate(vert, depth));
				addTriangles(points, triangles, sections);
				if (visitedSet.find(vert) != visitedSet.end()) {
					wasBranch = true;
					break;
				}
				visitedSet.insert(vert);
			}
			if (!wasBranch) {
				sections.add(nullptr);
				addTriangles(points, triangles, sections);
			}
			sections.clear();
		}
		for (auto i = triangles.begin(); i != triangles.end(); ++i) {
			if (i->normal().z < 0.0f) {
				i->flipRotation();
			}
		}
	}

	Vertex *findLowestJoiningVertex(Vertex *up, Vertex *down) {
		float min = std::numeric_limits<float>::max();
		Vertex *joining = nullptr;
		for (auto i = up->inbound().begin(); i != up->inbound().end(); ++i) {
			for (auto j = down->inbound().begin(); j != down->inbound().end(); ++j) {
				if (i->next->mVertex == j->next->mVertex && i->next->vertex().data().z < min){
					joining = i->next->mVertex;
					min = i->next->vertex().data().z;
				}
			}
		}
		return joining;
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

void TriangulatedTerrainGraph::carveRiverBeds(LastList &lastList) {
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		for (auto j = i->first->inbound().begin(); j != i->first->inbound().end(); ++j) {
			if (j->next->mVertex != i->first->data().down && j->next->mVertex->data().z < i->second.lowestZ) {
				i->second.lowestZ = j->next->mVertex->data().z;
			}
		}
		i->second.lowestZ -= sqrtf(i->first->data().flow) * lastList.flowMultiplier * 0.3f;
	}
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		i->first->data().z = i->second.lowestZ;
	}
}

void TriangulatedTerrainGraph::smoothRiverBeds(LastList &lastList) {
	std::vector<std::pair<Triangulation::Vertex *, Vector3>> adjusted;
	adjusted.reserve(lastList.size());
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		Vector3 last(i->second.last->position().x, i->second.last->position().y, i->second.last->data().z);
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
}

void TriangulatedTerrainGraph::fillLastList(LastList &lastList, float thresholdStandardDeviations) {
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
	lastList.reserve(mTriangulation->vertices().size());
	float totalDist = 0.0f;
	int maxFlow = 0;
	total = 0;
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		if (i->data().down && (i->data().flow - mean) >= target) {
			++total;
			totalDist += (i->data().down->position() - i->position()).magnitude();
			if (i->data().flow > maxFlow) {
				maxFlow = i->data().flow;
			}
			lastList.emplace_back(i->data().down, &*i);
		}
	}
	lastList.flowMultiplier = totalDist / (maxFlow * total);
}

void TriangulatedTerrainGraph::fillLastListInterpolated(TriangulatedTerrainGraph::RiverSections &verts, LastList &lastList) {
	std::map<Vector2, Triangulation::Vertex*> vertMap;
	std::set<Triangulation::Vertex*> visited;
	std::vector<Triangulation::Vertex*> sources, vettedSources;
	sources.reserve(verts.size());
	lastList.reserve(verts.size() << 1);
	for (auto i = mTriangulation->vertices().begin(); i != mTriangulation->vertices().end(); ++i) {
		vertMap.emplace(i->position(), &*i);
	}
	for (RiverSection &vecs : verts) {
		Triangulation::Vertex *up = vertMap[vecs.first], *down = vertMap[vecs.second];
		Triangulation::Vertex *middle = findLowestJoiningVertex(up, down);
		up->data().down = middle;
		middle->data().down = down;
		lastList.emplace_back(middle, up);
		lastList.emplace_back(down, middle);
		visited.insert(middle);
		visited.insert(down);
	}
	for (auto i = lastList.begin(); i != lastList.end(); ++i) {
		if (visited.find(i->second.last) == visited.end()) {
			sources.push_back(i->second.last);
		}
	}
	vettedSources.reserve(sources.size());
	for (Triangulation::Vertex *source : sources) {
		visited.clear();
		bool pass = true;
		for (Triangulation::Vertex *vert = source; pass && vert; vert = vert->data().down) {
			if (visited.find(vert) != visited.end()){
				pass = false;
			}
			else {
				visited.insert(vert);
			}
		}
		if (pass) {
			vettedSources.push_back(source);
		}
	}
	lastList.clear();
	for (Triangulation::Vertex *source : vettedSources) {
		while (source) {
			++source->data().flow;
			source = source->data().down;
		}
	}
	visited.clear();
	float totalDist = 0.0f;
	int maxFlow = 0, total = 0;
	for (Triangulation::Vertex *source : vettedSources) {
		float lowest = source->data().z;
		/*if (lowest > maxZ) {
			maxZ = lowest;
		}*/
		while (source->data().down) {
			if (source->data().down->data().z >= lowest) {
				source->data().down->data().z = lowest - FLT_EPSILON;
			}
			lastList.emplace_back(source->data().down, source);
			if (visited.find(source->data().down) != visited.end()) {
				break;
			}
			visited.insert(source->data().down);
			if (source->data().flow > maxFlow) {
				maxFlow = source->data().flow;
			}
			totalDist += (source->data().down->position() - source->position()).magnitude();
			++total;
			source = source->data().down;
		}
	}
	lastList.flowMultiplier = totalDist / (maxFlow * total);
}

Mesh &TriangulatedTerrainGraph::generateRiverMesh(LastList &lastList, Mesh &mesh) {
	std::vector<Triangle3> triangles;
	addTriangles(lastList, triangles);
	mesh.load(triangles);
	return mesh;
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