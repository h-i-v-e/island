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

namespace motu{
    
    template <class FaceData = EmptyData, class VertexData = EmptyData>
    class VoronoiGraph{
    public:
        
        typedef HalfEdge<Vector2, FaceData, VertexData> HalfEdge;
        typedef typename HalfEdge::Face Face;
        typedef typename HalfEdge::Vertex Vertex;
        
        typedef IterableObjectPool<HalfEdge> HalfEdges;
        typedef IterableObjectPool<Face> Faces;
        typedef IterableObjectPool<Vertex> Vertices;
        
        VoronoiGraph(size_t numVertices) : mHalfEdges(numVertices), mFaces(numVertices), mVertices(numVertices){}
        
        template <class Itr>
        void generate(const Itr &begin, const Itr &end){
            mHalfEdges.drain();
            mFaces.drain();
            mVertices.drain();
            std::set<Vector2> visited;
            std::vector<Vector2> vertexBuffer;
            VertexMap<Vector2, HalfEdges, Vertices, Faces> builder(&mHalfEdges, &mVertices, &mFaces);
            for (Itr i = begin; i != end; ++i){
                if (visited.find(i->/*vertex().*/position()) != visited.end()){
                    continue;
                }
                visited.insert(i->/*vertex().*/position());
                vertexBuffer.clear();
                for (auto &j : i->/*vertex().*/inbound()){
                    vertexBuffer.push_back(GetCircumcircleCentre(j));
                }
                builder.addPolygon(vertexBuffer.begin(), vertexBuffer.end());
            }
            mExternalFace = builder.bind();
        }

		void generate(const Mesh &mesh) {
			mHalfEdges.drain();
			mFaces.drain();
			mVertices.drain();
			std::vector<Vector2> vertexBuffer;
			VertexMap<Vector2, HalfEdges, Vertices, Faces> builder(&mHalfEdges, &mVertices, &mFaces);
			size_t numTris = mesh.triangles.size() / 3;
			std::vector<Vector2> centres(numTris);
			for (size_t i = 0; i != numTris; ++i) {
				size_t base = i * 3;
				centres[i] = Triangle(
					mesh.vertices[mesh.triangles[base]].asVector2(),
					mesh.vertices[mesh.triangles[base + 1]].asVector2(),
					mesh.vertices[mesh.triangles[base + 2]].asVector2()
				).findCircumcircleCentre();
			}
			const MeshTriangleMap &tris = mesh.triangleMap();
			for (size_t i = 0; i != mesh.vertices.size(); ++i) {
				auto j = tris.vertex(i);
				while (j.first != j.second) {
					int offset = *j.first++;
					vertexBuffer.push_back(centres[offset / 3]);
				}
				builder.addPolygon(vertexBuffer.begin(), vertexBuffer.end());
				vertexBuffer.clear();
			}
			mExternalFace = builder.bind();
		}
        
        const HalfEdges &halfEdges() const{
            return mHalfEdges;
        }
        
        HalfEdges &halfEdges(){
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
        
        const Face &externalFace() const{
            return *mExternalFace;
        }

		Face &externalFace() {
			return *mExternalFace;
		}
        
    private:
        HalfEdges mHalfEdges;
        Faces mFaces;
        Vertices mVertices;
        Face *mExternalFace;
        
        template <class TriangulationHalfEdge>
        static Vector2 GetCircumcircleCentre(const TriangulationHalfEdge &edge){
            const TriangulationHalfEdge *next = edge.next;
            return Triangle(edge.vertex().position(), next->vertex().position(), next->next->vertex().position()).findCircumcircleCentre();
        }
    };
}

#endif /* voronoi_graph_h */
