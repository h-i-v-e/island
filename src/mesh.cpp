//
//  mesh.cpp
//  World Maker
//
//  Created by Jerome Johnson on 20/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "mesh.h"
#include "half_edge.h"
#include "iterable_releasable_object_pool.h"
#include <queue>
#include "plane.h"
#include <vector>
#include <map>
#include "triangle3.h"
#include <algorithm>
#include <cassert>
#include "bounding_rectangle.h"
#include "brtree.h"
#include "triangle.h"
#include <array>
#include "bounding_box.h"
#include <unordered_map>
#include <unordered_set>
#include "util.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"

using namespace motu;

namespace{

	typedef BRTree<size_t> RefRtree;

	inline void interpolated(const Vector3 &endA, const Vector3 &endB, float hitTime, Vector3 &out) {
		out = endA + ((endB - endA) * hitTime);
	}

	inline void emplacePreserveRotation(const Vector3 normal, std::vector<Triangle3> &out, const Vector3 &a, const Vector3 &b, const Vector3 &c) {
		if ((b - a).cross(c - b).dot(normal) < 0.0f) {
			out.emplace_back(c, b, a);
		}
		else {
			out.emplace_back(a, b, c);
		}
	}

	template<class Tri, class InOut>
	bool scanBounds(const Tri &tri, float y, InOut &in, InOut &out){
        in.vertex.x = std::numeric_limits<float>::max();
        out.vertex.x = std::numeric_limits<float>::min();
        int found = 0;
		for (size_t i = 0; i != 3; ++i) {
			const Vector3 &a = tri.vertices[i], &b = tri.vertices[(i + 1) % 3];
			Edge edge(a.x, a.y, b.x, b.y);
            Vector2 direction(edge.direction());
            float x = !almostZero(direction.y) ? (((y * direction.x) - (edge.endA.y * direction.x))/ direction.y) + edge.endA.x : edge.endA.x;
            if (Edge::between(edge.endA.x, edge.endB.x, x)){
				float denom = edge.endB.x - edge.endA.x;
				float hitAt = !almostZero(denom) ? (x - edge.endA.x) / denom : 0.0f;
				InOut io;
				io.interpolate(tri, i, (i + 1) % 3, hitAt);
                if (x < in.vertex.x){
					in = io;
                }
                if (x > out.vertex.x){
                    out = io;
                }
                ++found;
            }
            if (found == 2){
                return true;
            }
        }
        return false;
    }
    
	template <class Raster, class Tri>
    void fillTriangle(Raster &grid, const Tri &tri){
        float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::min();
		for (size_t i = 0; i != 3; ++i) {
            float y = tri.vertices[i].y;
            if (y < minY){
                minY = y;
            }
            if (y > maxY){
                maxY = y;
            }
        }
        int h = grid.height(), w = grid.width();
        float stepSize = 1.0f / static_cast<float>(h);
        for (int y = minY * h, yend = std::min(static_cast<int>(maxY * h), h - 1); y <= yend; ++y){
            if (y < 0){
                continue;
            }
            float fy = static_cast<float>(y) * stepSize;
            Tri::InOut in, out;
            if (scanBounds(tri, fy, in, out)){
                int x = in.vertex.x * w, xend = out.vertex.x * w;
                float denom = xend - x;
                if (denom == 0.0f){
					if (x >= 0 && x < w) {
						in.assignTo(grid(x, y));
					}
                    continue;
                }
                Tri::InOut step(in, out, denom);
                while (x < 0){
					in += step;
                    ++x;
                }
                xend = std::min(xend, w - 1);
                while (x <= xend){
                    in.assignTo(grid(x, y));
					in += step;
                    ++x;
                }
            }
        }
    }

	struct TriWithNormals : public Triangle3WithNormals {
		struct InOut : public Mesh::VertexAndNormal {
			InOut() {}

			InOut(const InOut &in, const InOut &out, float denom) {
				vertex = (out.vertex - in.vertex) / denom;
				normal = (out.normal - in.normal) / denom;
			}

			InOut &operator+=(const InOut &add) {
				vertex += add.vertex;
				normal += add.normal;
				return *this;
			}

			void interpolate(const Triangle3WithNormals &tri, size_t endA, size_t endB, float at) {
				interpolated(tri.vertices[endA], tri.vertices[endB], at, vertex);
				interpolated(tri.normals[endA], tri.normals[endB], at, normal);
			}

			void assignTo(Mesh::VertexAndNormal &van) const{
				van = *this;
			}
		};
	};

	struct Tri : public Triangle3 {
		struct InOut {
			Vector3 vertex;

			InOut() {}

			InOut(const InOut &in, const InOut &out, float denom) {
				vertex = (out.vertex - in.vertex) / denom;
			}

			InOut &operator+=(const InOut &add) {
				vertex += add.vertex;
				return *this;
			}

			void interpolate(const Triangle3 &tri, size_t endA, size_t endB, float at) {
				interpolated(tri.vertices[endA], tri.vertices[endB], at, vertex);
			}

			void assignTo(Vector3 &v) const{
				v = vertex;
			}
		};
	};

	void splitTriangles(const Plane &divider, std::vector<Triangle3> &in, std::vector<Triangle3> &out) {
		size_t i = out.size();
		for (const Triangle3 &tri : in) {
			tri.slice(divider, out);
		}
		/*while (i != out.size()) {
			if (out[i].normal().dot(Vector3::unitZ()) < 0.0f) {
				out[i].flipRotation();
			}
			++i;
		}*/
	}

	inline Triangle3 getTriangle(const Mesh &mesh, size_t offset) {
		return Triangle3(
			mesh.vertices[mesh.triangles[offset]],
			mesh.vertices[mesh.triangles[offset + 1]],
			mesh.vertices[mesh.triangles[offset + 2]]
		);
	}

	struct Flip {
		Vector3 a, b, normal;
		bool complete;

		Flip(const Vector3 &vec, const Vector3 &normal) : a(vec), normal(normal), complete(false) {
		}
	};

	typedef std::pair<uint32_t, uint32_t> OffsetPair;

	struct OffsetPairHasher {
		size_t operator()(const OffsetPair &pair) const {
			return (pair.first * 1610612741) ^ (pair.second * 805306457);
		}
	};

	typedef std::unordered_map<OffsetPair, Flip, OffsetPairHasher> FlipMap;
	typedef std::vector<Triangle3> Triangles;

	void tesselateTriangle(const Mesh &mesh, uint32_t offset, FlipMap &flips, Triangles &out) {
		Triangle3 tri(getTriangle(mesh, offset));
		Vector3 norm(tri.normal());
		Vector3 centre(tri.baricentre());
		Spline splines[3];
		tri.getSplines(splines);
		for (int j = 0; j != 3; ++j) {
			if (splines[j].direction().squareMagnitude() < FLT_EPSILON) {
				out.push_back(tri);
				return;
			}
		}
		for (size_t j = 0; j != 3; ++j) {
			OffsetPair spline(mesh.triangles[offset + j], mesh.triangles[offset + ((j + 1) % 3)]);
			bool swapped = spline.second < spline.first;
			if (swapped) {
				std::swap(spline.first, spline.second);
			}
			auto k = flips.find(spline);
			if (k != flips.end()) {
				k->second.b = centre;
				k->second.complete = true;
			}
			else {
				flips.emplace_hint(k, spline, Flip(centre, norm));
			}
		}
	}

	typedef std::pair<uint32_t, uint32_t> OffsetPair;

	struct PerimeterHasher {
		size_t operator()(const OffsetPair &pair) const {
			return (pair.first * 1610612741) ^ (pair.second * 805306457);
		}
	};

	struct PerimeterSet : public std::unordered_set<uint32_t>{
		typedef std::unordered_set<OffsetPair, PerimeterHasher> EdgeSet;

		void addEdge(const Mesh &mesh, EdgeSet &edgeSet, uint32_t endA, uint32_t endB) {
			endA = mesh.triangles[endA];
			endB = mesh.triangles[endB];
			if (endB < endA) {
				std::swap(endA, endB);
			}
			OffsetPair op(endA, endB);
			auto i = edgeSet.find(op);
			if (i == edgeSet.end()) {
				edgeSet.emplace_hint(i, op);
			}
			else {
				edgeSet.erase(i);
			}
		}

		PerimeterSet(const Mesh &mesh) {
			EdgeSet edgeSet;
			edgeSet.reserve(mesh.triangles.size());
			for (uint32_t i = 2; i < mesh.triangles.size(); i += 3) {
				addEdge(mesh, edgeSet, i - 2, i - 1);
				addEdge(mesh, edgeSet, i - 1, i);
				addEdge(mesh, edgeSet, i, i - 2);
			}
			reserve(edgeSet.size());
			for (const OffsetPair &op : edgeSet) {
				insert(op.first);
				insert(op.second);
			}
		}
	};
}

Mesh &Mesh::tesselate(Mesh &mesh) const {
	FlipMap flips;
	flips.reserve(triangles.size());
	std::vector<Triangle3> out;
	out.reserve(triangles.size() * 3);
	for (size_t i = 2; i < triangles.size(); i += 3) {
		tesselateTriangle(*this, i - 2, flips, out);
	}
	for (auto i = flips.begin(); i != flips.end(); ++i) {
		if (i->second.complete) {
			emplacePreserveRotation(i->second.normal, out, i->second.a, i->second.b, vertices[i->first.first]);
			emplacePreserveRotation(i->second.normal, out, i->second.a, i->second.b, vertices[i->first.second]);
		}
		else{
			emplacePreserveRotation(i->second.normal, out, vertices[i->first.first], i->second.a, vertices[i->first.second]);
		}
	}
	mesh.load(out);
	return mesh;
}

void Mesh::load(std::vector<Triangle3> &tris) {
	std::unordered_map<Vector3, size_t, Hasher<Vector3>> vertexMap;
	size_t numVerts = tris.size() + 2;
	vertexMap.reserve(numVerts);
	triangles.reserve(tris.size());
	vertices.reserve(numVerts);
	for (auto i = tris.begin(); i != tris.end(); ++i) {
		for (size_t j = 0; j != 3; ++j) {
			const Vector3 &vert = i->vertices[j];
			auto k = vertexMap.find(vert);
			if (k != vertexMap.end()) {
				triangles.push_back(k->second);
			}
			else {
				size_t pos = vertices.size();
				vertices.push_back(vert);
				triangles.push_back(pos);
				vertexMap.emplace(vert, pos);
			}
		}
	}
}

void Mesh::calculateNormals() {
	normals.reserve(vertices.size());
	MeshTriangleMap triMap(*this);
	for (int i = 0; i != vertices.size(); ++i) {
		Vector3 normal = Vector3::zero();
		auto tris = triMap.vertex(i);
		while (tris.first != tris.second) {
			int offset = *(tris.first++);
			normal += Triangle3(vertices[triangles[offset]], vertices[triangles[offset + 1]], vertices[triangles[offset + 2]]).normal();
		}
		normal.normalize();
		normals.push_back(normal);
	}
}

void Mesh::smooth() {
	MeshEdgeMap edges(*this);
	PerimeterSet perimeter(*this);
	Vector3 *moved = new Vector3[vertices.size()];
	for (size_t i = 0; i != vertices.size(); ++i) {
		if (perimeter.find(i) != perimeter.end()) {
			moved[i] = vertices[i];
			continue;
		}
		Vector3 total(vertices[i]);
		std::pair<const uint32_t*, const uint32_t*> offsets(edges.vertex(i));
		float count = static_cast<float>((offsets.second - offsets.first) + 1);
		while (offsets.first != offsets.second) {
			total += vertices[*(offsets.first++)];
		}
		moved[i] = total / count;
	}
	std::copy(moved, moved + vertices.size(), vertices.data());
	delete[] moved;
}

void Mesh::rasterize(Grid<VertexAndNormal> &raster) const{
	TriWithNormals tri;
	for (size_t i = 0; i + 2 < triangles.size();) {
		bool valid = true;
		for (size_t j = 0; j != 3; ++j, ++i) {
			size_t offset = triangles[i];
			tri.vertices[j] = vertices[offset];
			if ((tri.normals[j] = normals[offset]).z < 0.0f) {
				tri.normals[j] *= -1.0f;
			}
		}
		if (valid) {
			fillTriangle(raster, tri);
		}
	}
}

void Mesh::rasterize(Grid<Vector3> &raster) const {
	Tri tri;
	for (size_t i = 0; i + 2 < triangles.size();) {
		for (size_t j = 0; j != 3; ++j, ++i) {
			size_t offset = triangles[i];
			tri.vertices[j] = vertices[offset];
		}
		fillTriangle(raster, tri);
	}
}

Mesh::Edges &Mesh::edges(Edges &edges) const {
	for (size_t i = 2; i < triangles.size(); i += 3) {
		edges.emplace(triangles[i - 2], triangles[i - 1]);
		edges.emplace(triangles[i - 1], triangles[i]);
		edges.emplace(triangles[i], triangles[i - 2]);
	}
	return edges;
}

Mesh &Mesh::slice(const BoundingBox &bounds, Mesh &out) const {
	std::vector<Triangle3> inside, selection, split;
	inside.reserve(triangles.size() / 3);
	for (size_t i = 2; i < triangles.size(); i += 3) {
		Triangle3 tri(getTriangle(*this, i - 2));
		if (bounds.intersects(tri)) {
			if (bounds.contains(tri)) {
				inside.push_back(tri);
			}
			else {
				split.push_back(tri);
			}
		}
	}
	Plane planes[6];
	bounds.getPlanes(planes);
	for (size_t i = 0; i != 6; ++i) {
		selection.clear();
		for (const Triangle3 &tri : split) {
			if (tri.intersects(planes[i])) {
				selection.push_back(tri);
			}
		}
		splitTriangles(planes[i], selection, split);
	}
	for (const Triangle3 &tri : split) {
		if (bounds.contains(tri)) {
			inside.push_back(tri);
		}
	}
	out.load(inside);
	return out;
}
