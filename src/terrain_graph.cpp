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

void TerrainGraph::generateTiles(std::default_random_engine &gen, size_t numTiles, int relaxations){
    std::vector<Vector2> points;
    points.reserve(numTiles);
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
	/*points.emplace_back(0.0f, 0.0f);
	points.emplace_back(0.0f, 0.5f);
	points.emplace_back(0.0f, 1.0f);
	points.emplace_back(0.5f, 1.0f);
	points.emplace_back(1.0f, 1.0f);
	points.emplace_back(1.0f, 0.5f);
	points.emplace_back(1.0f, 0.0f);
	points.emplace_back(0.5f, 0.0f);*/
    for (int i = 0; i != numTiles; ++i){
        points.emplace_back(dis(gen), dis(gen));
    }
    for (int i = 0;;){
        //DalauneyTriangulation<> triangulation(points);
		mTriangulation = new Triangulation(points);
        //voronoi.generate(triangulation.halfEdges().begin(), triangulation.halfEdges().end());
        if (i++ == relaxations){
            break;
        }
        points.clear();
        for (auto j = faces().begin(); j != faces().end(); ++j){
            points.push_back(j->calculateCentroid());
        }
		delete mTriangulation;
    }
	std::vector<Triangle3> triangles;
	triangles.reserve(mTriangulation->faces().size());
	for (auto i = mTriangulation->faces().begin(); i != mTriangulation->faces().end(); ++i) {
		const HalfEdge *j = i->halfEdge;
		triangles.emplace_back(j->vertex().position(), j->next().vertex().position(), j->next().next().vertex().position());
	}
	
}
