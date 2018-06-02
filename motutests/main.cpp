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
#include "island.h"
#include "mesh_decimator.h"
#include "bounding_box.h"
#include "rivers.h"
#include "mesh_edge_map.h"
#include "colour.h"
#include "sea_erosian.h"
#include "mesh_triangle_map.h"
#include "height_map.h"
#include "matrix4.h"
#include "unity.h"

using namespace motu;
using namespace std;

uint32_t clamp255(float f) {
	if (f < -127.0f) {
		f = -127.0f;
	}
	if (f > 127.0f) {
		f = 127.0f;
	}
	return static_cast<uint32_t>(127.0f + f);
}

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

inline float byteToFloat(uint32_t val, uint32_t shift) {
	static float mul = 1.0f / 127.5f;
	uint32_t mask = 0xff << shift;
	return (static_cast<float>((val & mask) >> shift) - 127.5f) * mul;
}

TEST_CASE("Continent", "Continent"){
	return;
    random_device rd;
    default_random_engine rand(rd());
	Island::Options options;
	Island island(rand, options);
	Raster raster(2048, 2048);
	raster.fill(0xffffff);
	Vector3 sun(0.0f, 0.5f, 1.0f);
	sun.normalize();
	Grid<Vector3> normals(2048, 2048);
	island.lod(2).rasterizeNormalsOnly(normals);
	Island::NormalAndOcclusianMap nam(2048, 2048);
	island.generateNormalAndOcclusianMap(nam);
	std::cout << island.riverMeshes().size() << std::endl;
	HeightMap hm(2048, 2048);
	hm.load(island.lod(0));
	static float occlusianMul = 1.0f / 255.0f;
	for (int i = 0, j = 2048 * 2048; i != j;  ++i) {
		//raster.data()[i] = nom.data()[i];
		if (hm.data()[i] < hm.seaLevel()) {
			raster.data()[i] = 0;
			continue;
		}
		uint32_t nac = nam.data()[i];
		Vector3 normal(byteToFloat(nac, 16), byteToFloat(nac, 8), byteToFloat(nac, 0));
		normal = (normal + normals.data()[i]) * 0.5f;
		//const Vector3 &normal = normals.data()[i];
		float occlusian = (nac >> 24) * occlusianMul;
		Vector3 colour = Vector3(1.0f, 1.0f, 1.0f) * sun.dot(normal) * occlusian;
		raster.data()[i] = toColour32(colour);
	}



	//Mesh mesh;
	//island.lod(2).slice(BoundingBox(Vector3(0.0f, 0.0f, -0.02f), Vector3(1.0f, 1.0f, 1.0f)), mesh);
	//mesh.transform(Matrix4::scale(4.0f) * Matrix4::translate(Vector3(-0.25f, -0.25f, 0.0f)));

	/*HeightMap hm(512, 512);
	hm.load(mesh);
	for (int y = 0; y != 512; ++y) {
		for (int x = 0; x != 512; ++x) {
			uint32_t gun = static_cast<uint32_t>(255.0f * hm(x, y));
			raster(x, y) = gun | (gun << 8) | (gun << 16) | 0xff000000;
		}
	}*/
	/*MeshEdgeMap mep(mesh);
	for (int i = 0; i != mesh.vertices.size(); ++i) {
		auto j = mep.vertex(i);
		const Vector2 &a = mesh.vertices[i].asVector2();
		while (j.first != j.second) {
			raster.draw(Edge(a, mesh.vertices[*j.first++].asVector2()), 0xffffffff);
		}
	}*/

    int rasterLength = raster.length() * 3;
    std::vector<unsigned char> buffer;
    buffer.reserve(rasterLength);
    const unsigned char *raw = reinterpret_cast<const unsigned char*>(raster.data());
    for (int i = 0, j = raster.length() << 2; i != j; i += 4){
        buffer.push_back(raw[i + 2]);
        buffer.push_back(raw[i + 1]);
        buffer.push_back(raw[i]);
    }
    //std::copy(raw, raw + rasterLength, std::back_inserter(buffer));
    writePNG("/Users/jerome/test.png", buffer, 2048, 2048);
}

TEST_CASE("unity", "unity") {
	MotuOptions options;
	options.coastalSlopeMultiplier = 1.0f;
	options.erosianPasses = 16;
	options.maxRiverGradient = 0.001f;
	options.maxZ = 0.1f;
	options.noiseMultiplier = 0.0005f;
	options.riverDepth = 0.002f;
	options.riverSourceSDThreshold = 1.0f;
	options.slopeMultiplier = 1.1f;
	options.waterRatio = 0.4f;
	void *handle = CreateMotu(666, &options);
	ExportHeightMap *ehm = CreateHeightMap(handle, 1024);
	ExportQuantisedRiverArray uvs;
	CreateQuantisedRiverMeshes(handle, ehm, &uvs);
	std::cout << uvs.length << std::endl;
	ReleaseQuantisedRiverMeshes(&uvs);
	ReleaseHeightMap(ehm);
	ReleaseMotu(handle);
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

