//
//  voronoi_graph.h
//  World Maker
//
//  Created by Jerome Johnson on 21/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//
#ifndef voronoi_graph_h
#define voronoi_graph_h

#include "iterable_object_pool.h"
#include "half_edge.h"
#include <vector>
#include "triangle.h"

namespace worldmaker{
    
    template <class FaceData = EmptyData, class VertexData = EmptyData>
    class VoronoiGraph{
    public:
        typedef HalfEdge<FaceData, VertexData> HalfEdgeType;
        typedef IterableObjectPool<HalfEdgeType> HalfEdges;
        typedef IterableObjectPool<typename HalfEdgeType::Face> Faces;
        typedef IterableObjectPool<typename HalfEdgeType::Vertex> Vertices;
        
        VoronoiGraph(size_t numVertices, size_t numFaces) : mHalfEdges(numVertices), mFaces(numFaces), mVertices(numVertices) {}
        
        template <class Itr>
        void generate(const Itr &begin, const Itr &end){
            mHalfEdges.drain();
            mFaces.drain();
            std::set<Vector2> visited;
            std::vector<Vector2> vertexBuffer;
            typename HalfEdgeType::Builder builder;
            for (Itr i = begin; i != end; ++i){
                if (visited.find(i->vector2) != visited.end()){
                    continue;
                }
                visited.insert(i->vector2);
                if (i->fullyConnected()){
                    vertexBuffer.clear();
                    for (auto &j : i->vertex().inbound()){
                        vertexBuffer.push_back(GetCircumcircleCentre(j));
                    }
                    builder.add(HalfEdgeType::fromPolygon(vertexBuffer.begin(), vertexBuffer.end(), mHalfEdges, mFaces.allocate()));
                }
            }
            builder.construct(mVertices);
        }
        
        const HalfEdges &halfEdges() const{
            return mHalfEdges;
        }
        
        const Faces &faces() const{
            return mFaces;
        }
        
        Faces &faces(){
            return mFaces;
        }
        
        const Vertices &vertices() const{
            return mVertices;
        }
        
        Vertices &vertices(){
            return mVertices;
        }
        
    private:
        HalfEdges mHalfEdges;
        Faces mFaces;
        Vertices mVertices;
        
        template <class TriangulationHalfEdge>
        static Vector2 GetCircumcircleCentre(const TriangulationHalfEdge &edge){
            const TriangulationHalfEdge *next = edge.next;
            return Triangle(edge.vector2, next->vector2, next->next->vector2).findCircumcircleCentre();
        }
    };
}

#endif /* voronoi_graph_h */
