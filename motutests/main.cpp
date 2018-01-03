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

using namespace motu;
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

/*void DrawPerimeter(const HalfEdge<Vector2> *h, Raster &raster){
    HalfEdge<Vector2>::Set visited;
    for (h = h->nextPerimeterEdge(visited); h; h = h->nextPerimeterEdge(visited)){
        raster.draw(Edge(h->vertex().position(), h->next->vertex().position()), 0xff0000ff);
    }
}*/

TEST_CASE("Continent", "Continent"){
    random_device rd;
    default_random_engine rand(rd());
    Continent continent(rand, 4096, 2, 0.05f);
    //continent.generateTiles(2);
    continent.generateSeasAndLakes(rand, 0.5f);
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

/*template<class Vertex>
Vector3 fromVertex(const Vertex &vertex) {
	return Vector3(vertex.position().x, vertex.position().y, vertex.data());
}

TEST_CASE("Decimate", "Decimate") {
	random_device rd;
	default_random_engine rand(rd());
	uniform_real_distribution<float> dis(0.0f, 1.0f);
	std::vector<Vector2> points;
	points.reserve(16);
	for (size_t i = 0; i != 16; ++i) {
		points.emplace_back(dis(rand), dis(rand));
	}
	DalauneyTriangulation<EmptyData, float> tri(points);
	for (auto i = tri.vertices().begin(); i != tri.vertices().end(); ++i) {
		i->data() = dis(rand);
	}
	std::vector<Triangle3> triangles;
	triangles.reserve(14);

	for (auto i = tri.faces().begin(); i != tri.faces().end(); ++i) {
		triangles.emplace_back(
			fromVertex(i->halfEdge()->vertex()),
			fromVertex(i->halfEdge()->next->vertex()),
			fromVertex(i->halfEdge()->next->next->vertex())
		);
	}
	Mesh mesh;
	mesh.load(triangles);
	mesh.decimate(8);
	Raster raster(1024, 1024);
	raster.fill(0xffffffff);
	raster.draw(mesh, 0x00000000);
	int rasterLength = raster.length() * 3;
	std::vector<unsigned char> buffer;
	buffer.reserve(rasterLength);
	const unsigned char *raw = reinterpret_cast<const unsigned char*>(raster.data());
	for (int i = 0, j = raster.length() << 2; i != j; i += 4) {
		buffer.push_back(raw[i]);
		buffer.push_back(raw[i + 1]);
		buffer.push_back(raw[i + 2]);
	}
	//std::copy(raw, raw + rasterLength, std::back_inserter(buffer));
	writePNG("/Users/jerome/test.png", buffer, 1024, 1024);
}*/


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

