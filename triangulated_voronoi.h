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
        struct FData{
            bool sea;
        };
        
        struct VData;
        
        typedef HalfEdge<FData, VData> HalfEdge;
        typedef typename HalfEdge::Face Face;
        typedef typename HalfEdge::Vertex Vertex;
        typedef IterableObjectPool<HalfEdge> HalfEdges;
        typedef IterableObjectPool<Vertex> Vertices;
        typedef IterableObjectPool<Face> Faces;
        
        struct VData{
            float z;
            Vector3 normal;
        };
        
    private:
        HalfEdges mHalfEdges;
        Vertices mVertices;
        int maxFlow;
        
        struct FaceAllocator{
            Faces faces;
            bool sea;
            
            FaceAllocator(int allocationBlockSize) : faces(allocationBlockSize){}
            
            Face *allocate(){
                Face *face = faces.allocate();
                face->data().sea = sea;
                return face;
            }
        };
        
        FaceAllocator mFaces;
        
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
            /*maxZ *= 0.1;
            Random randomness;
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                if (i->data().z != 0.0f){
                    continue;
                }
                bool isFlat = true;
                int seaCount = 0, landCount = 0;
                for (auto j = i->inbound().begin(); isFlat && j != i->inbound().end(); ++j){
                    if (j->next->vertex().data().z != 0.0f){
                        isFlat = false;
                    }
                    if (j->face().data().sea){
                        ++seaCount;
                    }
                    else{
                        ++landCount;
                    }
                }
                if (landCount > 0 && (isFlat || seaCount > landCount)){
                    i->data().z = randomness.get() * maxZ;
                    std::cout << i->data().z << std::endl;
                }
            }*/
        }
        
        static Vector3 vertexToVector3(const Vertex &vertex){
            return Vector3(vertex.position().x, vertex.position().y, vertex.data().z);
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
        
        /*void setFlow(){
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                i->data().flow = 0;
                float minZ = 0.0f;
                Vertex *down = nullptr;
                Vector3 me(i->position().x, i->position().y, i->data().z);
                for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                    Vector3 target(j->next->vertex().position().x, j->next->vertex().position().y, j->next->vertex().data().z);
                    Vector3 shift(target - me);
                    float z = shift.z / shift.squareMagnitude();
                    if (z < minZ){
                        minZ = z;
                        down = &j->next->vertex();
                    }
                }
                i->data().down = down;
            }
            maxFlow = 0;
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                if (i->data().down){
                    int flow = ++i->data().down->data().flow;
                    if (flow > maxFlow){
                        maxFlow = flow;
                    }
                }
            }
        }*/
        
        static Vertex *findLowestVertex(Face &face){
            Vertex *down = nullptr;
            float lowest = std::numeric_limits<float>::max();
            for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
                Vertex *vertex = &i->vertex();
                float z = vertex->data().z;
                if (z < lowest){
                    lowest = z;
                    down = vertex;
                }
            }
            return down;
        }
        
        static Vertex *findLowestVertex(Vertex &vertex){
            Vertex *down = nullptr;
            float lowest = vertex.data().z;
            for (auto i = vertex.inbound().begin(); i != vertex.inbound().end(); ++i){
                Vertex *vertex = &i->next->vertex();
                float z = vertex->data().z;
                if (z < lowest){
                    lowest = z;
                    down = vertex;
                }
            }
            return down;
        }
        
        static void trackRaindrop(Face &face, float carryCapacity){
            float carrying = 0.0f, velocity = 0.0f;
            Vertex *last = findLowestVertex(face);
            for (Vertex *down = findLowestVertex(*last); down; down = findLowestVertex(*down)){
                Vector3 direction(down->position().x, down->position().y, down->data().z);
                direction -= Vector3(last->position().x, last->position().y, last->data().z);
                float newVelocity = (velocity + direction.z) * 0.5f;
                float canCarry = newVelocity * carryCapacity;
                last->data().z += carrying - canCarry;
                carrying = canCarry;
                velocity = newVelocity;
                last = down;
            }
            last->data().z += carrying;
        }
        
    public:
        TriangulatedVoronoi(const VoronoiGraph &graph) : mHalfEdges(graph.vertices().size() * 6), mFaces(graph.halfEdges().size()), mVertices((graph.vertices().size() << 1) + graph.faces().size()){
            VertexMap<HalfEdges, Vertices, FaceAllocator> builder(&mHalfEdges, &mVertices, &mFaces);
            std::map<Vector2, float> zMap;
            for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
                mFaces.sea = i->data().sea;
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
            builder.bind();
            for (auto i = mVertices.begin(); i != mVertices.end(); ++i){
                auto j = zMap.find(i->position());
                i->data().z = j != zMap.end() ? j->second : notSet();
            }
            interpolateZValues();
            computeNormals();
            //setFlow();
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
            computeNormals();
            //setFlow();
        }
        
        void erode(int rainDrops, float carryCapacity){
            Random randomness;
            typedef BRTree<Face*> Tree;
            Tree tree(mFaces.faces.size());
            for (auto i = mFaces.faces.begin(); i != mFaces.faces.end(); ++i){
                tree.add(i->bounds(), &*i);
            }
            typename Tree::Values values;
            for (int i = 0; i != rainDrops; ++i){
                Vector2 dropPos(randomness.vector2());
                auto j = tree.containing(dropPos, values);
                while (j.first != j.second){
                    if ((*j.first)->contains(dropPos)){
                        trackRaindrop(**j.first, carryCapacity);
                    }
                    ++j.first;
                }
            }
            computeNormals();
        }
        
        Faces &faces(){
            return mFaces.faces;
        }
        
        const Faces &faces() const{
            return mFaces.faces;
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
