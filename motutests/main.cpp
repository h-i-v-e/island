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
#include "dalauney.h"
#include "graph.h"

using namespace motu;
using namespace std;

void sortPerimeter(std::vector<Vector2> &in) {
	Vector2 centre = Vector2::zero();
	for (size_t i = 0; i < in.size(); ++i) {
		centre += in[i];
	}
	centre /= static_cast<float>(in.size());
	std::sort(in.begin(), in.end(), [&centre](const Vector2 &a, const Vector2 &b) {
		return Triangle(centre, a, b).isClockwise();


		/*if (axlen >= 0.0f && bxlen < 0.0f) {
			assert(ca.dot(cb) < 0.0f);
			return false;
		}
		if (axlen < 0.0f && bxlen >= 0.0f) {
			assert(ca.dot(cb) > 0.0f);
			return true;
		}*/
		/*if (axlen == 0.0f && bxlen == 0.0f) {
		if (a.y - centre.y >= 0.0f || b.y - centre.y >= 0.0f)
		return a.y > b.y;
		return b.y > a.y;
		}*/


		// compute the cross product of vectors (center -> a) x (center -> b)
		/*float dot = ca.dot(cb);
		if (dot < 0.0f) {
			//a is to the right of b
			return true;
		}
		if (dot > 0.0f) {
			//b is to the right of a
			return false;
		}*/
		//return dot < 0.0f;
		// points a and b are on the same line from the center
		// check which point is closer to the center
		/*int d1 = (a.x - centre.x) * (a.x - centre.x) + (a.y - centre.y) * (a.y - centre.y);
		int d2 = (b.x - centre.x) * (b.x - centre.x) + (b.y - centre.y) * (b.y - centre.y);
		return d1 > d2;*/
		//return ca.sqrMagnitude() < cb.sqrMagnitude();
	});
}

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

/*TEST_CASE("Voronoi", "Voronoi") {
	static int numVerts = 1024;
	Mesh mesh;
	std::vector<Vector3> verts(numVerts, Vector3::zero());
	for (int i = 0; i != numVerts; ++i) {
		*reinterpret_cast<Vector2*>(verts.data() + i) = randomVector2();
	}
	createDalauneyMesh(verts, mesh);
	Graph<Vector3> graph;
	GraphBuilder<Vector3> gb(graph);

}*/

/*TEST_CASE("Continent", "Continent"){
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
	for (auto i = island.riverMeshes().begin(); i != island.riverMeshes().end(); ++i) {
		raster.draw(**i, 0x00000000);
	}
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
}*/

TEST_CASE("unity", "unity") {
	MotuOptions options;
	options.coastalSlopeMultiplier = 2.0f;
	options.maxZ = 0.05f;
	options.noiseMultiplier = 0.0005f;
	options.slopeMultiplier = 1.0f;
	options.waterRatio = 0.5f;
	void *handle = CreateMotu(2018, &options);
	Vector3ExportArray trees;
	ExportDecoration decoration;
	GetDecoration(handle, &decoration);
	TreeMeshPrototypes offsets;
	offsets.length = decoration.trees.length;
	TreeMeshPrototype *data = new TreeMeshPrototype[offsets.length];
	for (int i = 0; i != offsets.length; ++i) {
		data[i].offset = i;
		data[i].scale = 1.0f / 8192.0f;
	}
	offsets.prototypes = data;
	ExportTreeBillboardsArray etb;
	//SetLogFile("C:\\Users\\Jerome\\motu.log");
	CreateTreeBillboards(handle, &offsets, &etb);
	std::cout << "Out: " << etb.octants[0].mesh.vertices.length << std::endl;
	delete[] data;
	ReleaseTreeBillboards(&etb);
	/*SaveMotu(handle, "/Users/jerome/test.dat");
	ReleaseMotu(handle);*/
	//void *handle = LoadMotu("/Users/jerome/test.dat");
//	std::cout << "Made motu" << std::endl;
	/*HeightMap hm(1024, 1024);
	std::fill(hm.data(), hm.data() + 1024 * 1024, 0.0f);
	ExportMeshWithUVArray ema;
	CreateForestMeshesLod1(handle, 16, 0.01f, &ema);
	for (int i = 0; i != ema.length; ++i) {
		reinterpret_cast<const Mesh*>(ema.data[i].handle)->rasterize(hm);
	}
	ReleaseForestMeshesLod1(&ema);
	hm.normalise();
	std::vector<uint8_t> buffer;
	buffer.reserve(1024 * 1024 * 3);
	for (const float *i = hm.data(), *j = hm.data() + 1024 * 1024; i != j; ++i) {
		uint32_t gun = static_cast<uint32_t>(*i * 254.0f);
		buffer.push_back(gun);
		buffer.push_back(gun);
		buffer.push_back(gun);
	}

	writePNG("/Users/jerome/test.png", buffer, 1024, 1024);*/
	ExportHeightMapWithSeaLevel *ehm = CreateHeightMap(handle, 8192);
	reinterpret_cast<HeightMap*>(ehm)->normalise();
	Raster raster(8192, 8192);
	for (size_t i = 0, j = 8192 * 8192; i != j; ++i) {
		uint32_t gun = static_cast<uint32_t>(ehm->data[i] * 254.0f);
		raster.data()[i] = (gun << 16) | (gun << 8) | gun;
	}
	ReleaseHeightMap(ehm);
	raster.draw(reinterpret_cast<const Island*>(handle)->riverMesh(), 0x00ff0000);
	std::vector<uint8_t> buffer;
	buffer.reserve(8192 * 8192 * 3);
	for (auto i = raster.data(), j = raster.data() + raster.length(); i != j; ++i) {
		uint32_t val = *i;
		buffer.push_back(val & 0xff);
		buffer.push_back((val >> 8) & 0xff);
		buffer.push_back(val >> 16);
	}
	writePNG("/Users/jerome/test.png", buffer, 8192, 8192);
	/*const Raster &raster = *reinterpret_cast<Island*>(handle)->image;
	std::vector<uint8_t> buffer;
	buffer.reserve(raster.length() * 3);
	writePNG("/Users/jerome/test.png", buffer, raster.width(), raster.height());*/
	ReleaseMotu(handle);
}


/*TEST_CASE("makepolygon", "makepolygon") {
	std::vector<Vector2> points;
	RandomVector2 rand;
	for (size_t i = 0; i != 10; ++i) {
		points.push_back(rand());
	}
	sortPerimeter(points);
	Raster raster(1024, 1024);
	raster.fill(0xffffffff);
	for (size_t i = 1; i != 10; ++i) {
		raster.draw(Edge(points[i - 1], points[i]), 0);

	}
	int rasterLength = raster.length() * 3;
	std::vector<unsigned char> buffer;
	buffer.reserve(rasterLength);
	const unsigned char *raw = reinterpret_cast<const unsigned char*>(raster.data());
	for (int i = 0, j = raster.length() << 2; i != j; i += 4) {
		buffer.push_back(raw[i + 2]);
		buffer.push_back(raw[i + 1]);
		buffer.push_back(raw[i]);
	}
	//std::copy(raw, raw + rasterLength, std::back_inserter(buffer));
	writePNG("/Users/jerome/test.png", buffer, 1024, 1024);
}*/
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

