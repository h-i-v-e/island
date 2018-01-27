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

#include "grid.h"
#include "triangle3.h"


namespace motu{
	struct BoundingBox;

    struct Mesh{
        typedef std::vector<Vector3> Vertices;
        typedef std::vector<Vector3> Normals;
        typedef std::vector<uint32_t> Triangles;
		
		struct Edge : public std::pair<size_t, size_t> {
			Edge(size_t a, size_t b) : std::pair<size_t, size_t>(a, b) {}

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

		Mesh() {}

		void clear() {
			vertices.clear();
			normals.clear();
			triangles.clear();
		}

		void load(std::vector<Triangle3> &triangles);

		struct VertexAndNormal {
			Vector3 vertex, normal;
		};
        
        Vertices vertices;
        Normals normals;
        Triangles triangles;

		void calculateNormals();

		void smooth();

		void rasterize(Grid<VertexAndNormal> &) const;

		void rasterize(Grid<Vector3> &) const;

		Edges &edges(Edges &edges) const;

		bool manifold() const {
			Edges e;
			edges(e);
			size_t total = (triangles.size() / 3) + vertices.size() - (e.size() >> 1);
			return total == 2;
		}

		Mesh &slice(const BoundingBox &bounds, Mesh &out) const;

		Mesh &tesselate(Mesh &out) const;

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
    };
}

#endif /* mesh_h */
