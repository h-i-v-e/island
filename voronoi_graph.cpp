//
//  voronoi_graph.cpp
//  World Maker
//
//  Created by Jerome Johnson on 21/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "voronoi_graph.h"
#include "dalauney_triangulation.h"
#include "triangle.h"
#include <set>
#include <vector>

using namespace worldmaker;

namespace{
    Vector2 GetCircumcircleCentre(const HalfEdge<> &edge){
        const HalfEdge<> *next = edge.next;
        return Triangle(edge.vector2, next->vector2, next->next->vector2).findCircumcircleCentre();
    }
}

void VoronoiGraph::generate(const HalfEdgePool<HalfEdge<>> &triangulation){
    pool.drain();
    std::set<Vector2> visited;
    std::vector<Vector2> vertexBuffer;
    HalfEdge<>::Builder builder;
    for (const HalfEdge<> &edge : triangulation){
        if (visited.find(edge.vector2) != visited.end()){
            continue;
        }
        visited.insert(edge.vector2);
        if (edge.fullyConnected()){
            vertexBuffer.clear();
            for (const HalfEdge<> &j : edge.vertex().inbound()){
                vertexBuffer.push_back(GetCircumcircleCentre(j));
            }
            builder.add(HalfEdge<>::fromPolygon(vertexBuffer.begin(), vertexBuffer.end(), pool));
        }
    }
    builder.construct();
}

