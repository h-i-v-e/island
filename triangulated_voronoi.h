//
//  triangulated_voronoi.h
//  World Maker
//
//  Created by Jerome Johnson on 5/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef triangulated_voronoi_h
#define triangulated_voronoi_h

#include "half_edge.h"
#include "vector3.h"
#include "iterable_object_pool.h"
#include "random.h"

namespace worldmaker{
    template <class VoronoiGraph>
    class TriangulatedVoronoi{
    public:
        
        struct VData;
        
        typedef HalfEdge<Vector2, EmptyData, VData> HalfEdge;
        typedef typename HalfEdge::Face Face;
        typedef typename HalfEdge::Vertex Vertex;
        typedef IterableObjectPool<HalfEdge> HalfEdges;
        typedef IterableObjectPool<Vertex> Vertices;
        typedef IterableObjectPool<Face> Faces;
        
        struct VData{
            float z;
            Vector3 normal;
            bool cliff;
        };
        
    private:
        HalfEdges mHalfEdges;
        Vertices mVertices;
        Face *mExternalFace;
        int maxFlow;
        
        Faces mFaces;
        
        static constexpr float notSet(){
            return std::numeric_limits<float>::min();
        }
        
        void interpolateZValues(){
            std::vector<std::pair<Vertex *, float>> zValues;
            zValues.reserve(mVertices.size());
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                if (i->data().z == notSet()){
                    float totalZ = 0.0f;
                    int count = 0;
                    for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                        float z = j->next->vertex().data().z;
                        if (z != notSet()){
                            totalZ += z;
                            ++count;
                        }
                    }
                    zValues.push_back(std::make_pair(&*i, totalZ / count));
                }
            }
            float maxZ = 0.0f;
            for (auto i = zValues.begin(); i != zValues.end(); ++i){
                i->first->data().z = i->second;
                if (i->second > maxZ){
                    maxZ = i->second;
                }
            }
        }
        
        static Vector3 vertexToVector3(const Vertex &vertex){
            return Vector3(vertex.position().x, vertex.position().y, vertex.data().z);
        }
        
    public:
        TriangulatedVoronoi(const VoronoiGraph &graph) : mHalfEdges(graph.vertices().size() * 6), mFaces(graph.halfEdges().size()), mVertices((graph.vertices().size() << 1) + graph.faces().size()){
            VertexMap<Vector2, HalfEdges, Vertices, Faces> builder(&mHalfEdges, &mVertices, &mFaces);
            std::map<Vector2, float> zMap;
            for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
                if (&*i == &graph.externalFace()){
                    continue;
                }
                Vector2 centre(i->calculateCentroid());
                Vector2 vertices[3];
                Vector2 mid;
                auto last = i->halfEdges().begin();
                zMap.insert(std::make_pair(last->vertex().position(), last->vertex().data().z));
                for (auto j = ++i->halfEdges().begin(); j != i->halfEdges().end(); ++j){
                    zMap.insert(std::make_pair(j->vertex().position(), j->vertex().data().z));
                    mid = Edge(last->vertex().position(), j->vertex().position()).midPoint();
                    vertices[0] = centre;
                    vertices[1] = mid;
                    vertices[2] = last->vertex().position();
                    builder.addPolygon(vertices, vertices + 3);
                    vertices[0] = centre;
                    vertices[1] = j->vertex().position();
                    vertices[2] = mid;
                    builder.addPolygon(vertices, vertices + 3);
                    last = j;
                }
                mid = Edge(last->vertex().position(), i->halfEdges().begin()->vertex().position()).midPoint();
                vertices[0] = centre;
                vertices[1] = mid;
                vertices[2] = last->vertex().position();
                builder.addPolygon(vertices, vertices + 3);
                vertices[0] = centre;
                vertices[1] = i->halfEdges().begin()->vertex().position();
                vertices[2] = mid;
                builder.addPolygon(vertices, vertices + 3);
            }
            mExternalFace = builder.bind();
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                auto j = zMap.find(i->position());
                i->data().z = j != zMap.end() ? j->second : notSet();
            }
            interpolateZValues();
        }
        
        void computeNormals(){
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                Vector3 normal(0.0f, 0.0f, 0.0f);
                for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                    auto k = j->face().halfEdges().begin();
                    Vector3 a(vertexToVector3(k->vertex()));
                    Vector3 b(vertexToVector3((++k)->vertex()));
                    Vector3 c(vertexToVector3((++k)->vertex()));
                    normal += Triangle3(a, b, c).normal();
                }
                i->data().normal = normal.normalize();
            }
        }
        
        void smooth(){
            std::vector<std::pair<Vertex*, Vector3>> adjusted;
            adjusted.reserve(mVertices.size());
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                Vector3 total(0.0f, 0.0f, 0.0f);
                int count = 0;
                for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                    total += vertexToVector3(j->next->vertex());
                    ++count;
                }
                adjusted.push_back(std::make_pair(&*i, count == 0 ? total : total / static_cast<float>(count)));
            }
            for (auto i = adjusted.begin(); i != adjusted.end(); ++i){
                i->first->position().x = i->second.x;
                i->first->position().y = i->second.y;
                i->first->data().z = i->second.z;
            }
        }
        
        Faces &faces(){
            return mFaces;
        }
        
        const Faces &faces() const{
            return mFaces;
        }
        
        HalfEdges &halfEdges(){
            return mHalfEdges;
        }
        
        const HalfEdges &halfEdges() const{
            return mHalfEdges;
        }
        
        Vertices &vertices(){
            return mVertices;
        }
        
        const Vertices &vertices() const{
            return mVertices;
        }
        
        const Face &externalFace() const{
            return *mExternalFace;
        }
        
        static Triangle3WithNormals &copyTo(const Face &face, Triangle3WithNormals &tri){
            int i = 0;
            for (auto j = face.halfEdges().begin(); j != face.halfEdges().end(); ++j, ++i){
                tri.vertices[i] = vertexToVector3(j->vertex());
                tri.normals[i] = j->vertex().data().normal;
            }
            return tri;
        }
    };
}

#endif /* triangulated_voronoi_h */
