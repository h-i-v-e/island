#include <unordered_map>

#include "terrain.h"
#include "dalauney_triangulation.h"
#include "mesh.h"
#include "voronoi_graph.h"

using namespace motu;

namespace {
	typedef DalauneyTriangulation<> Triangulation;

	inline Vector3 vector2ToVector3(const Vector2 &vec) {
		return Vector3(vec.x, vec.y, 0.0f);
	}

	inline void copyTriangulationToMesh(const Triangulation &triangulation, Mesh &mesh) {
		std::vector<Triangle3> triangles;
		triangles.reserve(triangulation.faces().size());
		for (auto i = triangulation.faces().begin(); i != triangulation.faces().end(); ++i) {
			if (&*i != &triangulation.externalFace()) {
				const Triangulation::HalfEdgeType* edge = i->halfEdge();
				const Vector2 &a = edge->vertex().position(),
					&b = edge->next->vertex().position(),
					&c = edge->next->next->vertex().position();
				triangles.emplace_back(vector2ToVector3(a), vector2ToVector3(b), vector2ToVector3(c));
			}
		}
		mesh.load(triangles);
	}

	void createInitialMesh(const std::vector<Vector2> &verts, int tesselations, Mesh &mesh) {
		Triangulation triangulation(verts);
		copyTriangulationToMesh(triangulation, mesh);
		while (tesselations-- > 0) {
			mesh.tesselate();
		}
	}

	template <class Type>
	void setIndices(Type &type) {
		int counter = 0;
		for (auto i = type.begin(); i != type.end(); ++i) {
			i->data() = counter++;
		}
	}

	void setGraphIndices(Terrain &terrain) {
		setIndices(terrain.faces());
		setIndices(terrain.vertices());
	}
}

std::vector<Vector2> &Terrain::relax(std::vector<Vector2> &vecs) {
	Triangulation triangulation(vecs);
	vecs.clear();
	VoronoiGraph<> graph(triangulation.halfEdges().size() >> 1);
	graph.generate(triangulation.vertices().begin(), triangulation.vertices().end());
	for (const auto &i : graph.faces()) {
		Vector2 add(i.calculateCentroid());
		if (add.x < 0.0f) {
			add.x = 0.0f;
		}
		else if (add.x > 1.0f) {
			add.x = 1.0f;
		}
		if (add.y < 0.0f) {
			add.y = 0.0f;
		}
		else if (add.y > 1.0f) {
			add.y = 1.0f;
		}
		vecs.push_back(add);
	}
	return vecs;
}

Terrain::Terrain(const std::vector<Vector2> &verts, int tesselations) {
	Mesh mesh;
	createInitialMesh(verts, tesselations, mesh);
	mVertices = new Vertices(mesh.vertices.size());
	mHalfEdges = new HalfEdges(mesh.triangles.size());
	mFaces = new Faces(mesh.triangles.size() / 3);
	mExternalFace = mesh.createHalfEdgeGraph(*mVertices, *mHalfEdges, *mFaces);
	setGraphIndices(*this);
}

Terrain::Terrain(const Mesh &mesh) {
	mVertices = new Vertices(mesh.vertices.size());
	mHalfEdges = new HalfEdges(mesh.triangles.size());
	mFaces = new Faces(mesh.triangles.size() / 3);
	mExternalFace = mesh.createHalfEdgeGraph(*mVertices, *mHalfEdges, *mFaces);
	setGraphIndices(*this);
}

Mesh &Terrain::toMesh(Mesh &mesh) const{
	std::unordered_map<const Vertex*, uint32_t> vertexMap;
	vertexMap.reserve(mVertices->size());
	mesh.vertices.reserve(mVertices->size());
	mesh.triangles.reserve(mHalfEdges->size());
	uint32_t count = 0;
	for (auto i = mVertices->begin(); i != mVertices->end(); ++i) {
		vertexMap.emplace(&*i, count++);
		mesh.vertices.push_back(i->position());
	}
	for (auto i = mFaces->begin(); i != mFaces->end(); ++i) {
		if (&*i != mExternalFace) {
			const HalfEdge *a = i->halfEdge();
			do {
				mesh.triangles.push_back(vertexMap[a->mVertex]);
				a = a->next;
			} while (a != i->halfEdge());
		}
	}
	return mesh;
}