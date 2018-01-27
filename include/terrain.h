#ifndef TERRAIN_H
#define TERRAIN_H

#include <vector>

#include "half_edge.h"
#include "iterable_object_pool.h"
#include "vector3.h"

namespace motu {
	struct Mesh;

	class Terrain {
	public:
		typedef typename motu::HalfEdge<Vector3, int, int> HalfEdge;
		typedef typename HalfEdge::Vertex Vertex;
		typedef typename HalfEdge::Face Face;
		typedef typename IterableObjectPool<HalfEdge> HalfEdges;
		typedef typename IterableObjectPool<Vertex> Vertices;
		typedef typename IterableObjectPool<Face> Faces;

		static std::vector<Vector2> &relax(std::vector<Vector2> &);

		Terrain(const std::vector<Vector2> &vertices, int tessellations = 0);

		Terrain(const Mesh &mesh);

		~Terrain() {
			delete mHalfEdges;
			delete mVertices;
			delete mFaces;
		}

		Face &externalFace() {
			return *mExternalFace;
		}

		const Face &externalFace() const {
			return *mExternalFace;
		}

		HalfEdges &halfEdges() {
			return *mHalfEdges;
		}

		Vertices &vertices() {
			return *mVertices;
		}

		Faces &faces() {
			return *mFaces;
		}

		const HalfEdges &halfEdges() const{
			return *mHalfEdges;
		}

		const Vertices &vertices() const{
			return *mVertices;
		}

		const Faces &faces() const{
			return *mFaces;
		}

		Mesh &toMesh(Mesh &mesh) const;

		template <class ValType>
		std::vector<ValType> &initFaceData(std::vector<ValType> &data, const ValType &defaultVal) const {
			data.reserve(mFaces->size());
			for (int i = 0; i != mFaces->size(); ++i) {
				data.push_back(defaultVal);
			}
			return data;
		}

		template <class ValType>
		std::vector<ValType> &initVertexData(std::vector<ValType> &data, const ValType &defaultVal) const {
			data.reserve(mVertices->size());
			for (int i = 0; i != mVertices->size(); ++i) {
				data.push_back(defaultVal);
			}
			return data;
		}
	private:
		Face * mExternalFace;
		HalfEdges *mHalfEdges;
		Vertices *mVertices;
		Faces *mFaces;
	};
}

#endif