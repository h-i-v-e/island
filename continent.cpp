//
//  continent.cpp
//  World Maker
//
//  Created by Jerome Johnson on 25/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "continent.h"
#include "voronoi_graph.h"
#include "raster.h"
#include <random>
#include "dalauney_triangulation.h"
#include <map>
#include "simplex_noise.h"
#include <cmath>
#include <algorithm>
#include "noise_layer.h"
#include <stack>
#include "vector3.h"

using namespace worldmaker;
using namespace std;

namespace{
    //std::set<Continent::Face*> riverFaces;
    
    struct Random{
        random_device rd;
        mt19937 gen;
        uniform_real_distribution<float> dis;
        
        Random() : gen(rd()), dis(0.0f, 1.0f){
        }
        
        Vector2 vector2(){
            return Vector2(dis(gen), dis(gen));
        }
        
        uint32_t colour(){
            return (static_cast<uint32_t>(0xffffff * dis(gen)) << 8) | 0xff;
        }
        
        float get(){
            return dis(gen);
        }
    } randomness;
    
    struct FibonachiSequence{
        int a, b;
        
        FibonachiSequence() : a(0), b(1){}
        
        operator int() const{
            return b;
        }
        
        FibonachiSequence &operator++(){
            int next = a + b;
            a = b;
            b = next;
            return *this;
        }
    };
    
    void GetCentroids(const Continent::Graph &graph, std::vector<Vector2> &out){
        out.clear();
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (i->face().halfEdge() == &(*i)){
                out.push_back(i->face().calculateCentroid());
            }
        }
    }
    
    const Continent::Graph::HalfEdgeType &FindPerimeter(const Continent::Graph &graph){
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (i->onPerimeter()){
                return *i;
            }
        }
        return *graph.halfEdges().begin();
    }
    
    void MapSea(const Continent::Graph::HalfEdgeType &perimeter, std::set<Continent::Graph::HalfEdgeType::Face> &sea){
        stack<Continent::Graph::HalfEdgeType::Face> unvisited;
        unvisited.push(perimeter.face());
        while (!unvisited.empty()){
            Continent::Graph::HalfEdgeType::Face next(unvisited.top());
            unvisited.pop();
            for (const Continent::Graph::HalfEdgeType &edge : next.halfEdges()){
                if (edge.pair){
                    Continent::Graph::HalfEdgeType::Face face(edge.pair->face());
                    if (face.data().sea && sea.find(face) == sea.end()){
                        sea.insert(face);
                        unvisited.push(face);
                    }
                }
            }
        }
    }
    
    typedef std::vector<Triangle> TriangleList;
    
    template <class ColourFunc>
    void drawTriangle(Raster &raster, Vector3 *vertices, ColourFunc &func){
        raster.fillTriangle(vertices[0], vertices[1], vertices[2], func);
    }
    
    Vector3 vertexToVector3(const Continent::Vertex &vertex){
        return Vector3(vertex.position().x, vertex.position().y, vertex.data().z);
    }
    
    float computeAverageZ(const Continent::Face &face){
        /*float totalZ = 0.0;
        int total = 0;
        for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
            totalZ += i->vertex().data().z;
            ++total;
        }
        return totalZ / total;*/
        float minZ = numeric_limits<float>::max(), maxZ = numeric_limits<float>::min();
        for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
            float z = i->vertex().data().z;
            if (z < minZ){
                minZ = z;
            }
            if (z > maxZ){
                maxZ = z;
            }
        }
        return minZ + (randomness.get() * (maxZ - minZ));
    }
    
    template <class ColourFunc>
    void drawFace(Raster &raster, const Continent::Face &face, ColourFunc &func){
        Vector2 centre(face.calculateCentroid());
        float midZ = computeAverageZ(face);
        auto i = face.halfEdges().begin();
        auto last = i;
        Vector3 vertices[3];
        for (++i; i != face.halfEdges().end(); ++i){
            vertices[0] = vertexToVector3(last->vertex());
            vertices[1] = vertexToVector3(i->vertex());
            vertices[2].x = centre.x;
            vertices[2].y = centre.y;
            vertices[2].z = midZ;
            drawTriangle(raster, vertices, func);
            last = i;
        }
        vertices[0] = vertexToVector3(last->vertex());
        vertices[1] = vertexToVector3(face.halfEdges().begin()->vertex());
        vertices[2].x = centre.x;
        vertices[2].y = centre.y;
        vertices[2].z = midZ;
        drawTriangle(raster, vertices, func);
    }
    
    void RemoveLakes(Continent::Graph &graph){
        std::set<Continent::Graph::HalfEdgeType::Face> sea;
        MapSea(FindPerimeter(graph), sea);
        for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
            if (i->data().sea){
                if (sea.find(*i) == sea.end()){
                    i->data().sea = false;
                }
            }
        }
    }
    
    int ComputeDistanceToSea(Continent::Graph &graph){
        typedef Continent::Graph::HalfEdgeType::Vertex Vertex;
        typedef std::stack<Vertex> Stack;
        
        Stack circles[2];
        Stack *last = circles;
        Stack *next = circles + 1;
        
        FibonachiSequence fs;
        int distance = 0;
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            i->data().flow = numeric_limits<int>::max();
            for (auto j : i->inbound()){
                if (j.face().data().sea){
                    i->data().flow = 0;
                    last->push(*i);
                    break;
                }
            }
        }
        do {
            distance = ++fs;
            while (!last->empty()){
                Vertex vertex(last->top());
                last->pop();
                for (auto j = vertex.inbound().begin(); j != vertex.inbound().end(); ++j){
                    if (j->pair && distance < j->pair->vertex().data().flow){
                        j->pair->vertex().data().flow = distance;
                        next->push(j->pair->vertex());
                    }
                }
            }
            Stack *swap = last;
            last = next;
            next = swap;
        }while (!last->empty());
        return distance;
    }

    
    void setZValues(Continent::Graph &graph, float maxDistance){
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            i->data().z = static_cast<float>(i->data().flow) / maxDistance;
        }
    }
    
    void setDown(Continent::Vertex &vertex){
        if (vertex.data().z == 0.0f){
            vertex.data().down = nullptr;
            return;
        }
        Vector3 pos(vertex.position().x, vertex.position().y, vertex.data().z);
        float steepest = 1.1f;
        Continent::Vertex *winner = nullptr;
        auto itr = vertex.inbound();
        for (auto j = itr.begin(); j != itr.end(); ++j){
            if (j->face().data().sea){
                winner = nullptr;
                break;
            }
            if (j->next->vertex().data().down == &vertex){
                continue;
            }
            Vector3 other(j->next->vertex().position().x, j->next->vertex().position().y, j->next->vertex().data().z);
            Vector3 result(other - pos);
            result = result.normalized();
            if (result.z < steepest){
                steepest = result.z;
                winner = &j->pair->vertex();
            }
            else if (result.z == steepest && (j->next->vertex().position() - vertex.position()).magnitude() < (j->next->vertex().position() - winner->position()).magnitude()){
                steepest = result.z;
                winner = &j->pair->vertex();
            }
        }
        vertex.data().down = winner && steepest <= 0.0f ? winner : nullptr;
    }
    
    void setDown(Continent::Graph &graph){
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            setDown(*i);
        }
    }
    
    void calculateFlow(Continent::Graph &graph){
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            i->data().flow = 0;
        }
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            for (auto *down = i->data().down; down; down = down->data().down){
                ++down->data().flow;
            }
        }
    }
    
    struct SolidColourFill{
        uint32_t colour;
        
        SolidColourFill(uint32_t colour) : colour(colour){}
        
        uint32_t operator()(float x, float y, float z) const{
            return colour;
        }
    };
}

void Continent::tesselate(HalfEdges &edges){
    HalfEdges temp;
    copy(edges.begin(), edges.end(), back_inserter(temp));
    edges.clear();
    edges.reserve(temp.size() << 1);
    for (auto i = temp.begin(); i != temp.end(); ++i){
        edges.push_back(*i);
        Vertex *insert = graph.vertices().allocate();
        insert->data().z = ((*i)->vertex().data().z + (*i)->next->vertex().data().z) * 0.5f;
        insert->data().flow = (*i)->vertex().data().flow;
        insert->data().down = (*i)->vertex().data().down;
        (*i)->split(insert, graph.halfEdges());
        (*i)->vertex().data().down = insert;
        insert->position() += ((*i)->edge().normal() * ((randomness.get() - 0.5f) * 0.75f));
        edges.push_back((*i)->next);
    }
}

void Continent::generateRivers(int flowThreshold, int tesselations){
    HalfEdges ravines;
    for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
        if (i->data().down){
            auto down = Continent::Graph::HalfEdgeType::findHalfEdgeConnecting(i->inbound().begin(), i->inbound().end(), *i->data().down);
            if (down != i->inbound().end()){
                if (i->data().flow > flowThreshold){
                    rivers.push_back(&*down);
                }
                else{
                    ravines.push_back(&*down);
                }
            }
        }
    }
    tesselate(rivers);
    tesselate(ravines);
}

void Continent::generateTiles(int relaxations){
    std::vector<Vector2> points;
    points.reserve(numTiles);
    for (int i = 0; i != numTiles; ++i){
        points.push_back(randomness.vector2());
    }
    for (int i = 0;;){
        DalauneyTriangulation<> triangulation(points);
        graph.generate(triangulation.halfEdges().begin(), triangulation.halfEdges().end());
        if (++i == relaxations){
            break;
        }
        GetCentroids(graph, points);
    }
}

void Continent::generateSeasAndLakes(float waterRatio){
    NoiseLayer layers[4];
    float strength = 2.0f;
    for (int i = 0; i != 4; ++i){
        layers[i] = NoiseLayer(randomness.vector2(), strength);
        strength *= 4.0f;
    }
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        Vector2 centre = i->calculateCentroid();
        float noise = 0.0f;
        float dilute = 1.0f;
        for (int i = 0; i != 4; ++i){
            noise += layers[i].get(centre) / dilute;
            dilute *= 2.0f;
        }
        noise *= 1.0f - ((centre - Vector2(0.5f, 0.5f)).magnitude() * 1.5f);
        i->data().sea = noise < waterRatio;
    }
    RemoveLakes(graph);
    maxHeight = ComputeDistanceToSea(graph);
    setZValues(graph, maxHeight);
    setDown(graph);
    calculateFlow(graph);
    generateRivers(6, 1);
}

void Continent::draw(Raster &raster) const{
    SolidColourFill seaFill(0x00ff7700);
    struct{
        uint32_t operator()(float x, float y, float z){
            return 0x00009900 | static_cast<uint32_t>(z * 255);
        }
    } landFill;
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        if (i->data().sea){
            //raster.fill(*i, 0x00ff7700);
            drawFace(raster, *i, seaFill);
        }
        else{
            drawFace(raster, *i, landFill);
        }
    }
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        for (auto j : i->halfEdges()){
            if (j.pair && j.pair->face().data().sea != i->data().sea){
                raster.draw(j.edge(), 0x00009999, 4);
            }
            auto down = HalfEdge::findHalfEdgeConnecting(j.vertex().inbound().begin(), j.vertex().inbound().end(), *j.vertex().data().down);
            if (down != j.vertex().inbound().end() && *down == j){
                raster.draw(j.edge(), 0x99999999, 4);
            }
        }
    }
    /*for (auto i = riverFaces.begin(); i != riverFaces.end(); ++i){
        raster.fill(**i, 0xffffffff);
    }*/
    for (auto i = rivers.begin(); i != rivers.end(); ++i){
        raster.draw((*i)->edge(), 0x00ff9999);
    }
}
