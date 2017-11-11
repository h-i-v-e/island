//
//  terrain_graph.cpp
//  World Maker
//
//  Created by Jerome Johnson on 11/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "terrain_graph.h"
#include "dalauney_triangulation.h"
#include <random>
#include "triangle3.h"
#include "spline.h"
#include "triangulated_voronoi.h"

using namespace worldmaker;

namespace{
    typedef TriangulatedVoronoi<TerrainGraph> TriangulatedTerrainGraph;
    
    static bool scanBounds(const TriangulatedTerrainGraph::Face &tri, float y, Vector3 &in, Vector3 &out){
        in.x = std::numeric_limits<float>::max();
        out.x = std::numeric_limits<float>::min();
        int found = 0;
        for (auto i = tri.halfEdges().begin(); i != tri.halfEdges().end(); ++i){
            Edge edge(i->edge());
            Vector2 direction(edge.direction());
            float x = direction.y != 0.0f ? (((y * direction.x) - (edge.endA.y * direction.x))/ direction.y) + edge.endA.x : edge.endA.x;
            if (Edge::between(edge.endA.x, edge.endB.x, x)){
                float hitAt = (x - edge.endA.x) / (edge.endB.x - edge.endA.x);
                float z = i->vertex().data().z;
                z += ((i->next->vertex().data().z - z) * hitAt);
                Vector3 v3(x, y, z);
                if (x < in.x){
                    in = v3;
                }
                if (x > out.x){
                    out = v3;
                }
                ++found;
            }
            if (found == 2){
                return true;
            }
        }
        return false;
    }
    
    void fillTriangle(TerrainGraph::VertexGrid &grid, const TriangulatedTerrainGraph::Face &tri){
        float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::min();
        for (auto i = tri.halfEdges().begin(); i != tri.halfEdges().end(); ++i){
            float y = i->vertex().position().y;
            if (y < minY){
                minY = y;
            }
            if (y > maxY){
                maxY = y;
            }
        }
        int h = grid.height(), w = grid.width();
        float stepSize = 1.0f / static_cast<float>(h);
        for (int y = minY * h, yend = maxY * h; y <= yend; ++y){
            if (y < 0 || y >= h){
                continue;
            }
            float fy = static_cast<float>(y) * stepSize;
            Vector3 in, out;
            if (scanBounds(tri, fy, in, out)){
                int x = in.x * w, xend = out.x * w;
                float denom = xend - x;
                if (denom == 0.0f){
                    continue;
                }
                Vector3 step((out - in) / denom);
                while (x < 0){
                    in += step;
                    ++x;
                }
                xend = std::min(xend, w - 1);
                while (x <= xend){
                    grid(x, y) = in;
                    in += step;
                    ++x;
                }
            }
        }
    }

}

void TerrainGraph::generateTiles(unsigned int randomSeed, size_t numTiles, int relaxations){
    std::vector<Vector2> points;
    points.reserve(numTiles);
    std::mt19937 gen(randomSeed);
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    for (int i = 0; i != numTiles; ++i){
        points.emplace_back(dis(gen), dis(gen));
    }
    for (int i = 0;;){
        DalauneyTriangulation<> triangulation(points);
        voronoi.generate(triangulation.halfEdges().begin(), triangulation.halfEdges().end());
        if (i++ == relaxations){
            break;
        }
        points.clear();
        for (auto j = faces().begin(); j != faces().end(); ++j){
            points.push_back(j->calculateCentroid());
        }
    }
}

TerrainGraph::VertexGrid &TerrainGraph::copyTo(VertexGrid &grid) const{
    TriangulatedTerrainGraph graph(*this);
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        fillTriangle(grid, *i);
    }
    return grid;
}
