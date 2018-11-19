//
//  mesh.h
//  World Maker
//
//  Created by Jerome Johnson on 11/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef mesh_h
#define mesh_h

#include <vector>
#include <set>
#include <unordered_set>
#include <memory>

#include "grid.h"
#include "triangle3.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"
#include "mesh_tesselator.h"

namespace motu{
	struct BoundingBox;
	struct Matrix4;

    struct Mesh{
        typedef std::vector<Vector3> Vertices;
        typedef std::vector<Vector3> Normals;
        typedef std::vector<int> Triangles;
		
		struct Edge : public std::pair<int, int> {
			Edge(int a, int b) : std::pair<int, int>(a, b) {}

			bool operator < (const Edge &other) const {
				if (first < other.first) {
					return true;
				}
				if (first > other.first) {
					return false;
				}
				return second < other.second;
			}
		};

		typedef std::set<Edge> Edges;

		struct PerimeterSet : public std::unordered_set<int> {
			typedef std::pair<int, int> OffsetPair;
			struct PerimeterHasher {
				size_t operator()(const OffsetPair &pair) const {
					return (pair.first * HASH_PRIME_A) ^ (pair.second * HASH_PRIME_B);
				}
			};
			typedef std::unordered_set<OffsetPair, PerimeterHasher> EdgeSet;

			EdgeSet edgeSet;
		};

		PerimeterSet &getPerimeterSet(PerimeterSet &) const;

		Mesh() {}

		void clear() {
			vertices.clear();
			normals.clear();
			triangles.clear();
		}

		void load(std::vector<Triangle3> &triangles);

		void load(std::vector<Triangle3WithNormals> &triangles);

		struct VertexAndNormal {
			Vector3 vertex, normal;
		};
        
        Vertices vertices;
        Normals normals;
        Triangles triangles;

		void calculateNormals();

		void smooth();

		void smoothIfPositiveZ();

		void smooth(const std::unordered_set<int> &exclude);

		void rasterize(Grid<VertexAndNormal> &) const;

		void rasterize(Grid<Vector3> &) const;

		void rasterizeNormalsOnly(Grid<Vector3> &) const;

		void translate(const Vector3 &by) {
			for (Vector3 &vert : vertices) {
				vert += by;
			}
		}

		//height map only
		void rasterize(Grid<float> &) const;

		Edges &edges(Edges &edges) const;

		Triangle3 triangle(int offset) const{
			return Triangle3(vertices[triangles[offset]], vertices[triangles[offset + 1]], vertices[triangles[offset + 2]]);
		}

		bool manifold() const {
			Edges e;
			edges(e);
			size_t total = (triangles.size() / 3) + vertices.size() - (e.size() >> 1);
			return total == 2;
		}

		Mesh &slice(const BoundingBox &bounds, Mesh &out) const;

		Mesh &sliceAndAddSkirts(const BoundingBox &bounds, Mesh &out) const;

		enum Side {
			TOP = 1,
			LEFT = 2,
			BOTTOM = 4,
			RIGHT = 8
		};

		Mesh &slice(const BoundingBox &bounds, uint8_t sides, const Mesh &clampMesh, Mesh &out) const;

		Mesh &tesselate() {
			return motu::tesselate(*this);
		}

		Mesh &tesselateIfPositiveZ() {
			return motu::tesselate(*this, 0.0f);
		}

		//indices are the same a original triangles and values are offsets
		//into tesselated vertices
		//void tesselateAndMapCentroids(std::vector<int> &centroids, Mesh &tesselated);

		BoundingBox &getMaxSquare(BoundingBox &out) const;

		Mesh &transform(const Matrix4 &);

		template<class VertexAllocator, class HalfEdgeAllocator, class FaceAllocator>
		typename FaceAllocator::ObjectType *createHalfEdgeGraph(VertexAllocator &vertexAllocator, HalfEdgeAllocator &halfEdgeAllocator, FaceAllocator &faceAllocator) const{
			VertexMap<Vector3, HalfEdgeAllocator, VertexAllocator, FaceAllocator> vmap(&halfEdgeAllocator, &vertexAllocator, &faceAllocator);
			Vector3 tri[3];
			for (size_t i = 0; i < triangles.size();) {
				for (size_t j = 0; j != 3; ++j) {
					tri[j] = vertices[triangles[i++]];
				}
				vmap.addPolygon(tri, tri + 3);
			}
			return vmap.bind();
		}

		const MeshEdgeMap &edgeMap() const{
			if (!mEdgeMap.get()) {
				mEdgeMap = std::shared_ptr<MeshEdgeMap>(new MeshEdgeMap(*this));
			}
			return *mEdgeMap;
		}

		const MeshTriangleMap &triangleMap() const {
			if (!mTriangleMap.get()) {
				mTriangleMap = std::shared_ptr<MeshTriangleMap>(new MeshTriangleMap(*this));
			}
			return *mTriangleMap;
		}

		void ensureNormals() {
			if (normals.size() != vertices.size()) {
				calculateNormals();
			}
		}

		void dirty() {
			mEdgeMap = nullptr;
			mTriangleMap = nullptr;
			normals.clear();
		}

		private:
			mutable std::shared_ptr<MeshEdgeMap> mEdgeMap;
			mutable std::shared_ptr<MeshTriangleMap> mTriangleMap;
    };

	struct MeshWithUV : public Mesh{
		typedef std::vector<Vector2> Uv;

		Uv uv;
	};
}

#endif /* mesh_h */
