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

using namespace motu;

namespace{

	typedef BRTree<size_t> RefRtree;

	void interpolated(const Vector3 &endA, const Vector3 &endB, float hitTime, Vector3 &out) {
		out = endA + ((endB - endA) * hitTime);
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
            float x = direction.y != 0.0f ? (((y * direction.x) - (edge.endA.y * direction.x))/ direction.y) + edge.endA.x : edge.endA.x;
            if (Edge::between(edge.endA.x, edge.endB.x, x)){
				float denom = edge.endB.x - edge.endA.x;
				float hitAt = denom != 0.0f ? (x - edge.endA.x) / (edge.endB.x - edge.endA.x) : 0.0f;
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
		while (i != out.size()) {
			if (out[i].normal().dot(Vector3::unitZ()) < 0.0f) {
				out[i].flipRotation();
			}
			++i;
		}
	}

	inline Triangle3 getTriangle(const Mesh &mesh, size_t offset) {
		return Triangle3(
			mesh.vertices[mesh.triangles[offset]],
			mesh.vertices[mesh.triangles[offset + 1]],
			mesh.vertices[mesh.triangles[offset + 2]]
		);
	}

	bool probeZ(const Mesh &mesh, const RefRtree &rtree, RefRtree::Values &rtreeBuffer, const Vector2 &pos, Vector3 &hit) {
		rtreeBuffer.clear();
		Spline probe(Vector3(pos.x, pos.y, 1.0f), Vector3(pos.x, pos.y, -1.0f));
		auto bounds = rtree.containing(pos, rtreeBuffer);
		/*while (bounds.first != bounds.second) {
			if (getTriangle(mesh, *bounds.first).intersection(probe, hit)) {
				return true;
			}
			++bounds.first;
		}*/
		for (auto i = bounds.first; i != bounds.second; ++i) {
			Triangle tri(getTriangle(mesh, *bounds.first).toTriangle2());
			Vector2 shift((tri.findCentroid() - pos).normalized() * FLT_EPSILON);
			Spline eps(probe + Vector3(shift.x, shift.y, 0.0f));
			if (getTriangle(mesh, *bounds.first).intersection(/*probe*/eps, hit)) {
				return true;
			}
		}
		/*for (auto i = bounds.first; i != bounds.second; ++i) {
			if (getTriangle(mesh, *bounds.first).toTriangle2().contains(pos)) {
				//getTriangle(mesh, *bounds.first).intersection(probe, hit);
				return true;
			}
		}*/
		return false;
	}
}

void Mesh::load(std::vector<Triangle3> &tris) {
	std::map<Vector3, size_t> vertexMap;
	triangles.reserve(tris.size());
	vertices.reserve(tris.size() + 2);
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
	std::multimap<size_t, Vector3> triangleNormals;
	for (size_t i = 2; i < triangles.size(); i += 3) {
		size_t ia = triangles[i - 2], ib = triangles[i - 1], ic = triangles[i];
		const Vector3 &a = vertices[ia], &b = vertices[ib], &c = vertices[ic];
		Vector3 normal((b - a).cross(c - b));
		triangleNormals.emplace(ia, normal);
		triangleNormals.emplace(ib, normal);
		triangleNormals.emplace(ic, normal);
	}
	normals.reserve(vertices.size());
	for (size_t i = 0; i != vertices.size(); ++i) {
		Vector3 total(0.0f, 0.0f, 0.0f);
		int count = 0;
		auto j = triangleNormals.equal_range(i);
		while (j.first != j.second) {
			total += j.first->second;
			++j.first;
			++count;
		}
		normals.push_back((total / count).normalize());
	}
}

void Mesh::smooth() {
	std::multimap<size_t, size_t> edges;
	std::vector<Vector3> movedVertices;
	movedVertices.reserve(vertices.size());
	for (size_t i = 2; i < triangles.size(); i += 3) {
		for (size_t j = i - 2; j <= i; ++j) {
			for (size_t k = i - 2; k <= i; ++k) {
				edges.emplace(triangles[k], triangles[j]);
			}
		}
	}
	for (size_t i = 0; i != vertices.size(); ++i) {
		int count = 0;
		Vector3 total(0.0f, 0.0f, 0.0f);
		auto j = edges.equal_range(i);
		while (j.first != j.second) {
			total += vertices[j.first->second];
			++j.first;
			++count;
		}
		movedVertices.push_back(total / count);
	}
	//vertices = movedVertices;
	//avoiding some memory fragmentation
	std::copy(movedVertices.data(), movedVertices.data() + movedVertices.size(), vertices.data());
}

void Mesh::rasterize(Grid<VertexAndNormal> &raster) const{
	TriWithNormals tri;
	for (size_t i = 0; i + 2 < triangles.size();) {
		for (size_t j = 0; j != 3; ++j, ++i) {
			size_t offset = triangles[i];
			tri.vertices[j] = vertices[offset];
			tri.normals[j] = normals[offset];
		}
		fillTriangle(raster, tri);
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
		Triangle3 tri(
			vertices[triangles[i - 2]],
			vertices[triangles[i - 1]],
			vertices[triangles[i]]
		);
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
