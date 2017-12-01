//
//  mesh.cpp
//  World Maker
//
//  Created by Jerome Johnson on 20/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "mesh.h"
#include "half_edge.h"
#include "iterable_object_pool.h"
#include <queue>
#include "plane.h"
#include <vector>
#include <map>
#include "triangle3.h"
#include <algorithm>

using namespace motu;

namespace{
    typedef HalfEdge<Vector3> HalfEdge;
    typedef IterableObjectPool<HalfEdge> HalfEdges;
    typedef IterableObjectPool<HalfEdge::Face> Faces;
    typedef IterableObjectPool<HalfEdge::Vertex> Verts;

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
    
    struct TriangleVertexIterator{
        const Mesh *mesh;
        int triangleIndex;
        
        TriangleVertexIterator(const Mesh *mesh, int triangleIndex) : mesh(mesh), triangleIndex(triangleIndex){}
        
        const Vector3 &operator*() const{
            return mesh->vertices[mesh->triangles[triangleIndex]];
        }
        
        const Vector3 *operator->() const{
            return &mesh->vertices[mesh->triangles[triangleIndex]];
        }
        
        bool operator != (const TriangleVertexIterator &other) const{
            return triangleIndex != other.triangleIndex;
        }
        
        bool operator == (const TriangleVertexIterator &other) const{
            return triangleIndex != other.triangleIndex;
        }
        
        TriangleVertexIterator &operator++(){
            ++triangleIndex;
            return *this;
        }
    };
    
    void constructHalfEdgeGraph(const Mesh &mesh, HalfEdges &halfEdges, Faces &faces, Verts &vertices){
        VertexMap<Vector3, HalfEdges, Verts, Faces> builder(&halfEdges, &vertices, &faces);
        for (int i = 0; i != mesh.triangles.size(); i += 3){
            builder.addPolygon(TriangleVertexIterator(&mesh, i), TriangleVertexIterator(&mesh, i + 3));
        }
        builder.bind();
    }
    
    struct CollapsePoint{
        HalfEdge::Vertex *vertA, *vertB;
        Vector3 collapseTo;
        float cost;
        
        CollapsePoint(){}
        
        CollapsePoint(HalfEdge::Vertex *vertA, HalfEdge::Vertex *vertB){
            if (vertA > vertB){
                this->vertA = vertB;
                this->vertB = vertA;
            }
            else{
                this->vertA = vertA;
                this->vertB = vertB;
            }
        }
    };
    
    typedef ObjectPool<CollapsePoint> CollapsePointPool;
    
    struct CollapsePointCompare{
        bool operator()(const CollapsePoint *a, const CollapsePoint *b){
            if (a->cost < b->cost){
                return true;
            }
            if (a->vertA < b->vertA){
                return true;
            }
            return a->vertB < b->vertB;
        }
    };
    
    struct SplineTest{
        Vector3 a, b;
        
        SplineTest(const HalfEdge &edge) : a(edge.vertex().position()), b(edge.next->vertex().position()){
            if (b < a){
                std::swap(a, b);
            }
        }
        
        bool operator<(const SplineTest &other){
            if (a < other.a){
                return true;
            }
            return b < other.b;
        }
    };
    
    struct QueueOrder{
        bool operator()(const CollapsePoint &a, const CollapsePoint &b){
            return a.cost < b.cost;
        }
    };
    
    Plane &toPlane(const HalfEdge::Face &face, Plane &plane){
        plane.point = face.halfEdge()->vertex().position();
        const Vector3 &next = face.halfEdge()->next->vertex().position();
        plane.normal = (next - plane.point).cross(face.halfEdge()->next->next->vertex().position() - next);
        return plane;
    }
    
    float distanceToPlanesAround(const HalfEdge::Vertex &vertex, Vector3 point){
        float total = 0.0f;
        Plane plane;
        for (auto i = vertex.inbound().begin(); i != vertex.inbound().end(); ++i){
            total += toPlane(i->face(), plane).distanceTo(point);
        }
        return total;
    }
    
    void findCollapsePoint(CollapsePoint &target){
        float a = distanceToPlanesAround(*target.vertB, target.vertA->position());
        float b = distanceToPlanesAround(*target.vertA, target.vertB->position());
        Vector3 mid(target.vertA->position() + ((target.vertB->position() - target.vertA->position()) * 0.5f));
        float c = distanceToPlanesAround(*target.vertB, mid) + distanceToPlanesAround(*target.vertA, mid);
        if (a < b){
            if (a < c){
                target.collapseTo = target.vertA->position();
                target.cost = a;
                return;
            }
        }
        else if (b < c){
            target.collapseTo = target.vertB->position();
            target.cost = b;
            return;
        }
        target.collapseTo = mid;
        target.cost = c;
    }
    
    
}
    
void Mesh::decimate(int num){
    /*typedef HalfEdge<Vector3> HalfEdge;
    HalfEdges halfEdges(triangles.size());
    Verts verts(vertices.size());
    Faces faces(triangles.size() / 3);
    CollapsePointPool collapsePoints(triangles.size());
    constructHalfEdgeGraph(*this, halfEdges, faces, verts);
    std::set<CollapsePoint*, CollapsePointCompare> waiting;
    std::multimap<Vector3, CollapsePoint*> vertMap;
    std::set<SplineTest> added;
    for (auto i = halfEdges.begin(); i != halfEdges.end(); ++i){
        SplineTest test(*i);
        if (added.find(test) != added.end()){
            continue;
        }
        added.insert(test);
        CollapsePoint *cp = collapsePoints.allocate(i->vertex(), i->next->vertex());
        findCollapsePoint(*cp);
        vertMap.emplace(cp->vertA->position(), cp);
        vertMap.emplace(cp->vertB->position(), cp);
        waiting.insert(cp);
    }
    std::vector<CollapsePoint *> update;
    while (num-- != 0){
        update.clear();
        CollapsePoint *cp = *waiting.begin();
        waiting.erase(waiting.begin());
        auto j = vertMap.equal_range(cp->vertA->position());
        for (auto k = j.first; k != j.second; ++k){
            update.push_back(k->second);
        }
        vertMap.erase(j.first, j.second);
        j = vertMap.equal_range(cp->vertB->position());
        for (auto k = j.first; k != j.second; ++k){
            update.push_back(k->second);
        }
        vertMap.erase(j.first, j.second);
        for (auto k = update.begin(); k != update.end(); ++k){
            waiting.erase(*k);
        }
        for (auto k = update.begin(); k != update.end(); ++k){
            if ((*k)->vertA == cp->vertA){
                (*k)->vertA = cp->collapseTo;
            }
        }
    }*/
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
