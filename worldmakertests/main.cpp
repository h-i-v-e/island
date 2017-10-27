//
//  main.cpp
//  worldmakertests
//
//  Created by Jerome Johnson on 3/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include <iostream>
#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "brtree.h"
#include <limits>
#include <random>
#include <algorithm>
#include "dalauney_triangulation.h"
#include "lodepng.h"
#include "raster.h"
#include "half_edge.h"
#include "voronoi_graph.h"
#include "continent.h"

using namespace worldmaker;
using namespace std;

struct RandomVector2{
    random_device rd;
    mt19937 gen;
    uniform_real_distribution<float> dis;
    
    RandomVector2() : gen(rd()), dis(0.0f, 1.0f){
    }
    
    Vector2 operator()(){
        return Vector2(dis(gen), dis(gen));
    }
    
    uint32_t colour(){
        return (static_cast<uint32_t>(0xffffff * dis(gen)) << 8) | 0xff;;
    }
} randomVector2;

void writePNG(const char* filename, std::vector<unsigned char>& image, unsigned width, unsigned height)
{
    std::vector<unsigned char> png;
    unsigned error = lodepng::encode(png, image, width, height, LCT_RGB);
    if(!error) lodepng::save_file(png, filename);
    
    //if there's an error,display it
    if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
}


/*TEST_CASE("Binary R-Tree", "BinaryRTree"){
    static int tests = 32;
    typedef BRTree<BoundingRectangle> Tree;
    Tree rtree(tests);
    BoundingRectangle rects[tests];
    Tree::Values values(tests);
    for (int i = 0; i != tests; ++i){
        rects[i].clear();
        rects[i].add(randomVector2());
        rects[i].add(randomVector2());
        rtree.add(rects[i], rects[i]);
    }
    cout << rtree.getDepth() << endl;
    for (int i = 0; i != tests; ++i){
        auto p = rtree.containing(rects[i].centre(), values);
        REQUIRE(find(p.first, p.second, rects[i]) != p.second);
    }
    for (int i = 0; i != tests; ++i){
        rtree.remove(rects[i], rects[i]);
        auto p = rtree.containing(rects[i].centre(), values);
        REQUIRE(find(p.first, p.second, rects[i]) == p.second);
        for (int j = i + 1; j != tests; ++j){
            auto p = rtree.containing(rects[j].centre(), values);
            REQUIRE(find(p.first, p.second, rects[j]) != p.second);
        }
    }
}*/

void DrawPerimeter(const HalfEdge<> *h, Raster &raster){
    HalfEdge<>::Set visited;
    for (h = h->nextPerimeterEdge(visited); h; h = h->nextPerimeterEdge(visited)){
        raster.draw(h->edge(), 0xff0000ff);
    }
}

TEST_CASE("Continent", "Continent"){
    Continent continent(4096);
    continent.generateTiles(2);
    continent.generateSeasAndLakes(0.5f);
    Raster raster(1024, 1024);
    raster.fill(0xffffffff);
    continent.draw(raster);
    int rasterLength = raster.length() * 3;
    std::vector<unsigned char> buffer;
    buffer.reserve(rasterLength);
    const unsigned char *raw = reinterpret_cast<const unsigned char*>(raster.data());
    for (int i = 0, j = raster.length() << 2; i != j; i += 4){
        buffer.push_back(raw[i]);
        buffer.push_back(raw[i + 1]);
        buffer.push_back(raw[i + 2]);
    }
    //std::copy(raw, raw + rasterLength, std::back_inserter(buffer));
    writePNG("/Users/jerome/test.png", buffer, 1024, 1024);
}

/*TEST_CASE("Triangulation", "Triangulation"){
    static int num = 1024;
    std::vector<Vector2> points;
    points.reserve(num);
    for (int i = 0; i != num; ++i){
        points.push_back(randomVector2());
    }
    std::cout << "Starting..." << std::endl;
    DalauneyTriangulation triangulation(points);
    std::cout << "triangulated... " << triangulation.halfEdge().onPerimeter() << std::endl;
    VoronoiGraph voronoi(num);
    voronoi.generate(triangulation.halfEdge());
    std::cout << "voronoid... " << voronoi.halfEdge().onPerimeter() << std::endl;
    Raster raster(1024, 1024);
    raster.fill(0xffffffff);
    HalfEdge::Set all;
    for (auto i : voronoi.halfEdge().getAllReachableHalfEdges(all)){
        //raster.draw(i->edge(), i->pair ? 0x000000ff : 0xff0000ff);
        raster.draw(i->edge(), 0x000000ff);
        if (i->fullyConnected()){
            raster.fill(i->face(), 0xff0000ff);
        }
    }
    int rasterLength = raster.length() << 2;
    std::vector<unsigned char> buffer;
    buffer.reserve(rasterLength);
    const unsigned char *raw = reinterpret_cast<const unsigned char*>(raster.data());
    std::copy(raw, raw + rasterLength, std::back_inserter(buffer));
    writePNG("/Users/jerome/test.png", buffer, 1024, 1024);
}*/

