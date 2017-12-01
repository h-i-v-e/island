//
//  terrain_graph.cpp
//  World Maker
//
//  Created by Jerome Johnson on 11/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "terrain_graph.h"
#include <random>
#include "triangle3.h"
#include "spline.h"
#include "triangulated_terrain_graph.h" 

using namespace motu;

namespace{
    
    void findFlow(TerrainGraph &tg){
        for (auto i = tg.vertices().begin(); i != tg.vertices().end(); ++i){
            i->data().flow = 0;
            if (i->data().z < 0.0f){
                continue;
            }
            float lowest = std::numeric_limits<float>::max();
            Vector3 pos(i->position().x, i->position().y, i->data().z);
            for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                if (&j->next->vertex() == &*i){
                    continue;
                }
                Vector3 ins(j->next->vertex().position().x, j->next->vertex().position().y, j->next->vertex().data().z);
                float z = (ins - pos).normalize().z;
                if (z < lowest){
                    i->data().down = &j->next->vertex();
                    lowest = z;
                }
            }
        }
        std::set<const TerrainGraph::Vertex *> visited;
        for (auto i = tg.vertices().begin(); i != tg.vertices().end(); ++i){
            visited.clear();
            for (TerrainGraph::Vertex *v = i->data().down; v; v = v->data().down){
                if (v->data().down && visited.find(v->data().down) != visited.end()){
                    float lowest = std::numeric_limits<float>::max();
                    Vector3 pos(v->position().x, v->position().y, v->data().z);
                    for (auto j = v->inbound().begin(); j != v->inbound().end(); ++j){
                        if (visited.find(&j->pair->vertex()) != visited.end()){
                            continue;
                        }
                        Vector3 ins(j->next->vertex().position().x, j->next->vertex().position().y, j->next->vertex().data().z);
                        float z = (ins - pos).normalize().z;
                        if (z < lowest){
                            v->data().down = &j->next->vertex();
                            lowest = z;
                        }
                    }
                }
				++v->data().flow;
				visited.insert(v);
            }
        }
    }
}

Rivers::Edges &TerrainGraph::findRivers(Rivers::Edges &edges, float thresholdStandardDeviations){
    findFlow(*this);
    int total = 0, count = 0;
    for (auto i = vertices().begin(); i != vertices().end(); ++i){
        int flow = i->data().flow;
        if (flow){
            total += flow;
            ++count;
        }
    }
    float mean = static_cast<float>(total) / count, variance = 0.0;
    for (auto i = vertices().begin(); i != vertices().end(); ++i){
        int flow = i->data().flow;
        if (flow){
            float v = (flow - mean);
            variance += v * v;
        }
    }
    variance /= total;
    float target = sqrtf(variance) * thresholdStandardDeviations;
    for (auto i = vertices().begin(); i != vertices().end(); ++i){
        if (i->data().down && (i->data().flow - mean) >= target){
            float z = i->data().down->data().z;
            if (z < 0.0f){
                Vector2 direction(i->data().down->position() - i->position());
                float toSea = (1.0f + ((0.0f - z) / (z - i->data().z)));
                edges.emplace_back(i->position(), i->position() + (direction * toSea));
            }
            else{
                edges.emplace_back(i->position(), i->data().down->position());
            }
        }
    }
    return edges;
}

void TerrainGraph::generateTiles(std::default_random_engine &gen, size_t numTiles, int relaxations){
    std::vector<Vector2> points;
    points.reserve(numTiles);
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

TerrainGraph::VertexGrid &TerrainGraph::copyTo(VertexGrid &grid, int smoothings) const{
    /*TriangulatedTerrainGraph graph(*this);
	Mesh mesh;
	graph.toMesh(mesh);
    while (smoothings--){
		mesh.smooth();
    }
	mesh.rasterize(grid);*/
    return grid;
}

void TerrainGraph::copyBackZValues(const VertexGrid &grid){
    for (auto i = voronoi.vertices().begin(); i != voronoi.vertices().end(); ++i){
        int x = i->position().x * grid.width();
        int y = i->position().y * grid.height();
        if (x < 0.0f || y < 0.0f){
            continue;
        }
        if (x >= grid.width()){
            x = grid.width() - 1;
        }
        if (y >= grid.height()){
            y = grid.height() - 1;
        }
        i->data().z = grid(x, y).z;
    }
}
