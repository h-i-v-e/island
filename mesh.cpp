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

using namespace worldmaker;

namespace{
    typedef HalfEdge<Vector3> HalfEdge;
    typedef IterableObjectPool<HalfEdge> HalfEdges;
    typedef IterableObjectPool<HalfEdge::Face> Faces;
    typedef IterableObjectPool<HalfEdge::Vertex> Verts;
    
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
