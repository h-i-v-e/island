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

using namespace worldmaker;
using namespace std;

namespace{
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
                    if (face.data().distanceToSea == 0 && sea.find(face) == sea.end()){
                        sea.insert(face);
                        unvisited.push(face);
                    }
                }
            }
        }
    }
    
    void RemoveLakes(Continent::Graph &graph){
        std::set<Continent::Graph::HalfEdgeType::Face> sea;
        MapSea(FindPerimeter(graph), sea);
        for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
            if (i->data().distanceToSea == 0){
                if (sea.find(*i) == sea.end()){
                    i->data().distanceToSea = -1;
                }
            }
        }
    }
    
    int ComputeDistanceToSea(Continent::Graph &graph){
        typedef Continent::Graph::HalfEdgeType::Face Face;
        typedef std::stack<Face> Stack;
        
        Stack circles[2];
        Stack *last = circles;
        Stack *next = circles + 1;
        
        int distance = 1;
        for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
            if (i->data().distanceToSea == 0){
                last->push(*i);
            }
        }
        do {
            while (!last->empty()){
                Face face(last->top());
                last->pop();
                auto j = face.halfEdge();
                do {
                    if (j->pair){
                        int d = j->pair->face().data().distanceToSea;
                        if (d == -1 || d > distance){
                            j->pair->face().data().distanceToSea = distance;
                            next->push(j->pair->face());
                        }
                    }
                    j = j->next;
                } while (j != face.halfEdge());
            }
            ++distance;
            last = next;
            next = &circles[distance & 1];
        }while (!last->empty());
        return distance - 1;
    }
    
    void setZValues(Continent::Graph &graph, float maxDistance){
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            int count = 0, total = 0;
            for (auto j : i->inbound()){
                total += j.face().data().distanceToSea;
                ++count;
            }
            float t = total, c = count;
            i->data().z = t / (c * maxDistance);
        }
    }
    
    inline Continent::Graph::HalfEdgeType::Vertex *GetDown(Continent::Graph::HalfEdgeType::Vertex &vert){
        return reinterpret_cast<Continent::Graph::HalfEdgeType::Vertex*>(vert.data().down);
    }
    
    inline const Continent::Graph::HalfEdgeType::Vertex *GetDown(const Continent::Graph::HalfEdgeType::Vertex &vert){
        return reinterpret_cast<const Continent::Graph::HalfEdgeType::Vertex*>(vert.data().down);
    }
    
    void setDown(Continent::Graph &graph){
        typedef Continent::Graph::HalfEdgeType::Vertex Down;
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            if (i->data().z == 0.0f){
                i->data().down = nullptr;
                continue;
            }
            Vector3 pos(i->position().x, i->position().y, i->data().z);
            float steepest = 1.1f;
            Down *winner(nullptr);
            auto itr = i->inbound();
            for (auto j = itr.begin(); j != itr.end(); ++j){
                if (j->face().data().distanceToSea == 0){
                    winner = nullptr;
                    break;
                }
                if (!j->pair || j->pair->vertex().data().down == &*i){
                    continue;
                }
                Vector3 other(j->pair->vector2.x, j->pair->vector2.y, j->pair->vertex().data().z);
                Vector3 result(other - pos);
                result = result.normalized();
                if (result.z < steepest){
                    steepest = result.z;
                    winner = const_cast<Down *>(&j->pair->vertex());
                }
                else if (result.z == steepest && (j->pair->vector2 - i->position()).magnitude() < (j->pair->vector2 - winner->position()).magnitude()){
                    steepest = result.z;
                    winner = const_cast<Down *>(&j->pair->vertex());
                }
            }
            i->data().down = winner && winner->data().z > 0.0f && steepest <= 0.0f ? winner : nullptr;
        }
    }
    
    void calculateFlow(Continent::Graph &graph){
        typedef Continent::Graph::HalfEdgeType::Vertex Down;
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            for (Down *down = GetDown(*i); down; down = GetDown(*down)){
                ++down->data().flow;
            }
        }
    }
    
    inline bool IsSea(const Continent::Graph::HalfEdgeType::Face &face){
        return face.data().distanceToSea == 0;
    }
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
        i->data().distanceToSea = noise < waterRatio ? 0 : -1;
    }
    RemoveLakes(graph);
    maxHeight = ComputeDistanceToSea(graph);
    setZValues(graph, maxHeight);
    setDown(graph);
    calculateFlow(graph);
}

void Continent::draw(Raster &raster) const{
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        if (i->data().distanceToSea == 0){
            raster.fill(*i, 0x00ff7700);
        }
        else{
            uint32_t gun = (i->data().distanceToSea * 255) / maxHeight;
            raster.fill(*i, 0x00009900 | gun);
        }
    }
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        for (auto j : i->halfEdges()){
            if (j.pair && IsSea(j.pair->face()) != IsSea(*i)){
                raster.draw(j.edge(), 0xffffffff, 4);
            }
            /*else{
                raster.draw(j.edge(), 0x99999999, 4);
            }*/
        }
    }
    for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
        if (i->data().flow > 48 && GetDown(*i)){
            raster.draw(Edge(i->position(), GetDown(*i)->position()), 0x00ff9999, 4);
        }
    }
}
