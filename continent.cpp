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
    
    void GetCentroids(const VoronoiGraph &graph, std::vector<Vector2> &out){
        out.clear();
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (i->face().halfEdge() == &(*i)){
                out.push_back(i->face().calculateCentroid());
            }
        }
    }
    
    const HalfEdge<> &FindPerimeter(const VoronoiGraph &graph){
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (i->onPerimeter()){
                return *i;
            }
        }
        return *graph.halfEdges().begin();
    }
    
    void MapSea(const HalfEdge<> &perimeter, const std::map<HalfEdge<>::Face, bool> &tiles, std::set<HalfEdge<>::Face> &sea){
        stack<HalfEdge<>::Face> unvisited;
        unvisited.push(perimeter.face());
        while (!unvisited.empty()){
            HalfEdge<>::Face next(unvisited.top());
            unvisited.pop();
            for (const HalfEdge<> &edge : next.halfEdges()){
                if (edge.pair){
                    HalfEdge<>::Face face(edge.pair->face());
                    if (edge.pair && sea.find(face) == sea.end() && tiles.find(face)->second){
                        sea.insert(face);
                        unvisited.push(face);
                    }
                }
            }
        }
    }
    
    void RemoveLakes(const VoronoiGraph &graph, std::map<HalfEdge<>::Face, bool> &tiles){
        std::set<HalfEdge<>::Face> sea;
        MapSea(FindPerimeter(graph), tiles, sea);
        for (auto i = tiles.begin(); i != tiles.end(); ++i){
            if (i->second && sea.find(i->first) == sea.end()){
                i->second = false;
            }
        }
    }
}

void Continent::generateTiles(int relaxations){
    std::vector<Vector2> points;
    points.reserve(numTiles);
    for (int i = 0; i != numTiles; ++i){
        points.push_back(randomness.vector2());
    }
    for (int i = 0;;){
        DalauneyTriangulation triangulation(points);
        graph.generate(triangulation.halfEdges());
        if (++i == relaxations){
            break;
        }
        GetCentroids(graph, points);
    }
    HalfEdge<>::Face::Set faces;
    for (const HalfEdge<> &edge : graph.halfEdges()){
        tiles.insert(make_pair(edge.face(), true));
    }
}

void Continent::generateSeasAndLakes(float waterRatio){
    NoiseLayer layers[4];
    float strength = 2.0f;
    for (int i = 0; i != 4; ++i){
        layers[i] = NoiseLayer(randomness.vector2(), strength);
        strength *= 4.0f;
    }
    
    for (auto i = tiles.begin(); i != tiles.end(); ++i){
        Vector2 centre = i->first.calculateCentroid();
        float noise = 0.0f;
        float dilute = 1.0f;
        for (int i = 0; i != 4; ++i){
            noise += layers[i].get(centre) / dilute;
            dilute *= 2.0f;
        }
        noise *= 1.0f - ((centre - Vector2(0.5f, 0.5f)).magnitude() * 1.5f);
        /*float distance = std::min((centre - Vector2(0.5f, 0.5f)).magnitude() * 2.0f, 1.0f);
        if (distance > 0.9f){
            noise *= (0.5f + (sinf((1.0f - distance) * 10.0f * M_PI) * 0.5f));
        }*/
        //std::cout << distance  << " => " << 0.5f + (cosf(distance * M_PI) * 0.5f) << std::endl;
        //noise *= (0.5f + (cosf(distance * M_PI) * 0.5f));
        i->second = noise < waterRatio;
    }
    RemoveLakes(graph, tiles);
}

void Continent::draw(Raster &raster) const{
    for (auto i : tiles){
        //std::cout << i.second << std::endl;
        raster.fill(i.first, i.second ? 0x00ff7700 : 0x00009900);
    }
    for (auto i : tiles){
        for (auto j : i.first.halfEdges()){
            if (j.pair && tiles.find(j.pair->face())->second != i.second){
                raster.draw(j.edge(), 0xffffffff, 4);
            }
            else{
                raster.draw(j.edge(), 0x99999999, 4);
            }
        }
    }
}
