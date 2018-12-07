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
#include "util.h"

namespace motu{
	struct BoundingBox;
	struct Matrix4;
	class Decoration;

    struct Mesh{
        typedef std::vector<Vector3> Vertices;
        typedef std::vector<Vector3> Normals;
        typedef std::vector<int> Triangles;

		typedef Triangle3WithNormals TriangleType;
		
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

		Mesh(const Mesh &mesh) : vertices(mesh.vertices), normals(mesh.normals), triangles(mesh.triangles){
		}

		Mesh &operator=(const Mesh &mesh) {
			vertices = mesh.vertices;
			normals = mesh.normals;
			triangles = mesh.triangles;
			return *this;
		}

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

		void calculateNormals(const MeshTriangleMap &);

		void smooth(const MeshEdgeMap &);

		void smoothIfPositiveZ(const MeshEdgeMap &);

		void smoothIfNegativeZ(const MeshEdgeMap &);

		void smooth(const MeshEdgeMap &mem, const Decoration &);

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

		friend std::ostream &operator<<(std::ostream &out, const Mesh &mesh) {
			motu::writeOutVector(out, mesh.vertices);
			motu::writeOutVector(out, mesh.triangles);
			return out;
		}

		friend std::istream &operator>>(std::istream &in, Mesh &mesh) {
			motu::readInVector(in, mesh.vertices);
			motu::readInVector(in, mesh.triangles);
			mesh.calculateNormals(mesh);
			return in;
		}
    };

	struct MeshWithUV : public Mesh{
		typedef std::vector<Vector2> Uv;
		typedef Triangle3WithNormalsAndUV TriangleType;

		Uv uv;

		MeshWithUV() {}

		MeshWithUV(const MeshWithUV &mesh) : Mesh(mesh), uv(mesh.uv) {}

		MeshWithUV &operator=(const MeshWithUV &mesh) {
			Mesh::operator=(mesh);
			uv = mesh.uv;
			return *this;
		}

		friend std::ostream &operator<<(std::ostream &out, const MeshWithUV &mesh) {
			out << *reinterpret_cast<const Mesh*>(&mesh);
			motu::writeOutVector(out, mesh.uv);
			return out;
		}

		friend std::istream &operator>>(std::istream &in, MeshWithUV &mesh) {
			in >> *reinterpret_cast<Mesh*>(&mesh);
			motu::readInVector(in, mesh.uv);
			return in;
		}

		MeshWithUV &slice(const BoundingBox &bounds, MeshWithUV &out) const;

		void load(std::vector<Triangle3WithNormalsAndUV> &triangles);

		MeshWithUV &tesselate() {
			return motu::tesselate(*this);
		}
	};
}

#endif /* mesh_h */
