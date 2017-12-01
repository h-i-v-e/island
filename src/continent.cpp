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
#include "random.h"
#include "dalauney_triangulation.h"
#include <map>
#include "simplex_noise.h"
#include <cmath>
#include <algorithm>
#include "noise_layer.h"
#include <stack>
#include "vector3.h"
#include "triangle3.h"
#include "triangulated_terrain_graph.h"
#include "erosian_map.h"

using namespace motu;
using namespace std;

namespace{
    
   /* struct ErosianFill{
        ErosianMap erosian;
        
        ErosianFill(int width, int height, float rate) : erosian(width, height, rate){
            erosian.zero();
        }
        
        uint32_t operator()(int x, int y, const Raster::Vector3WithNormal &vals){
            Vector3 assign(vals.x, vals.y, vals.z < -0.01f ? -0.01f : vals.z);
            Vector3 &target = erosian(x, y);
            target = assign;
            if (target.z != assign.z){
                target.z = 0.0f;
                std::cout << "Wo" << std::endl;
            }
            return 0;
        }
    };*/
    
    struct FibonachiSequence{
        int a, b;
        
        FibonachiSequence() : a(0), b(1){}
        
        operator int() const{
            return b;
        }
        
        FibonachiSequence &operator++(){
            int next = a + b;
            a = b;
            b = next;
            return *this;
        }
    };
    
    void MapSea(const TerrainGraph::HalfEdge &perimeter, std::set<TerrainGraph::Face> &sea){
        stack<TerrainGraph::Face> unvisited;
        unvisited.push(perimeter.face());
        while (!unvisited.empty()){
            TerrainGraph::Face next(unvisited.top());
            unvisited.pop();
            for (const TerrainGraph::HalfEdge &edge : next.halfEdges()){
                if (edge.pair){
                    TerrainGraph::Face face(edge.pair->face());
                    if (face.data().sea && sea.find(face) == sea.end()){
                        sea.insert(face);
                        unvisited.push(face);
                    }
                }
            }
        }
    }
    
    typedef std::vector<Triangle> TriangleList;
    
    template <class ColourFunc>
    void drawTriangle(Raster &raster, Vector3 *vertices, ColourFunc &func){
        raster.fillTriangle(vertices[0], vertices[1], vertices[2], func);
    }
    
    float computeAverageZ(const TerrainGraph::Face &face){
        float totalZ = 0.0;
        int total = 0;
        for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
            totalZ += i->vertex().data().z;
            ++total;
        }
        return totalZ / total;
    }
    
    typedef std::vector<Triangle3> Triangulation;
    
    Vector3 fromVertex(const TerrainGraph::Vertex &vertex){
        const Vector2 &vec = vertex.position();
        return Vector3(vec.x, vec.y, vertex.data().z);
    }
    
    void RemoveLakes(TerrainGraph &graph){
        std::set<TerrainGraph::Face> sea;
        MapSea(*graph.externalFace().halfEdge(), sea);
        for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
            if (i->data().sea){
                if (sea.find(*i) == sea.end()){
                    i->data().sea = false;
                }
            }
        }
    }
    
    struct DistanceToSea{
        bool isTarget(bool sea) const{
            return sea;
        }
        
        float &target(TerrainGraph::VertexData &data) const{
            return data.seaDistance;
        }
    };
    
    struct DistanceToLand{
        bool isTarget(bool sea) const{
            return !sea;
        }
        
        float &target(TerrainGraph::VertexData &data) const{
            return data.landDistance;
        }
    };
    
    template <class DistanceTo>
    float ComputeDistanceTo(TerrainGraph &graph, DistanceTo dt){
        typedef TerrainGraph::Vertex Vertex;
        typedef std::stack<Vertex> Stack;
        
        Stack circles[2];
        Stack *last = circles;
        Stack *next = circles + 1;

        //int lin = 0;
        //FibonachiSequence fib;
        float distance = 0.0f, add = 0.5f, addMul = 1.05f;
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            dt.target(i->data()) = numeric_limits<float>::max();
            for (auto j : i->inbound()){
                if (dt.isTarget(j.face().data().sea)){
                    dt.target(i->data()) = 0;
                    last->push(*i);
                    break;
                }
            }
        }
        do {
            //++lin;
            //++fib;
            //int distance = (fib + (lin * 15)) >> 4;
            add *= addMul;
            distance += add;
            while (!last->empty()){
                Vertex vertex(last->top());
                last->pop();
                for (auto j = vertex.inbound().begin(); j != vertex.inbound().end(); ++j){
                    if (j->pair && distance < dt.target(j->pair->vertex().data())){
                        dt.target(j->pair->vertex().data()) = distance;
                        next->push(j->pair->vertex());
                    }
                }
            }
            Stack *swap = last;
            last = next;
            next = swap;
        }while (!last->empty());
        return distance;//(fib + (lin * 15)) >> 4;
    }
    
    void setZValues(TerrainGraph &graph, float maxDistance, float maxZ){
        float mul = maxZ / maxDistance;
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            i->data().z = static_cast<float>(i->data().seaDistance - i->data().landDistance) * mul;
        }
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            if (i->data().z == 0.0f){
                int seaCount = 0, landCount = 0;
                for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                    float z = j->next->vertex().data().z;
                    if (z < 0.0f){
                        ++seaCount;
                    }
                    else if (z > 0.0f){
                        ++landCount;
                    }
                }
                if (seaCount > landCount){
                    i->data().z = landCount * mul;
                    i->data().cliff = true;
                    for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
                        float z = j->next->vertex().data().z;
                        if (z < 0.0f){
                            j->next->vertex().data().z -= mul;
                        }
                    }
                }
            }
        }
    }
    
    inline bool isCoastal(const TerrainGraph::HalfEdge &edge){
        return edge.pair && edge.face().data().sea != edge.pair->face().data().sea;
    }
    
    typedef std::set<const TerrainGraph::HalfEdge *> VisitedSet;
    
    Vector3 computeNormal(const TerrainGraph::Face *external, const TerrainGraph::Vertex &vertex){
        Vector3 total(0.0f, 0.0f, 0.0f);
        for (auto i = vertex.inbound().begin(); i != vertex.inbound().end(); ++i){
            if (&i->face() == external){
                continue;
            }
            Vector2 centre(i->face().calculateCentroid());
            Triangle3 tri(fromVertex(i->next->vertex()), fromVertex(vertex), Vector3(centre.x, centre.y, computeAverageZ(i->face())));
            Vector3 normal((tri.vertices[1] - tri.vertices[0]).cross(tri.vertices[2] - tri.vertices[0]));
            if (normal.z < 0.0f){
                normal *= -1.0f;
            }
            total += normal;
        }
        return total.normalize();
    }
    
    /*struct SolidColourFill{
        uint32_t colour;
        
        SolidColourFill(uint32_t colour) : colour(colour){}
        
        uint32_t operator()(const Raster::Vector3WithNormal &vals) const{
            return colour;
        }
    };*/
}

void Continent::computeNormals(){
    for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
        i->data().normal = computeNormal(&graph.externalFace(), *i);
    }
}

void Continent::generateSeasAndLakes(std::default_random_engine &rnd, float waterRatio){
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    maxZ = std::normal_distribution<float>(maxZ, maxZ * 0.2f)(rnd);
    NoiseLayer layers[4];
    float strength = 2.0f;
    for (int i = 0; i != 4; ++i){
        layers[i] = NoiseLayer(Vector2(dis(rnd), dis(rnd)), strength);
        strength *= 4.0f;
    }
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        if (&*i == &graph.externalFace()){
            continue;
        }
        Vector2 centre = i->calculateCentroid();
        float noise = 0.0f;
        float dilute = 1.0f;
        for (int i = 0; i != 4; ++i){
            noise += layers[i].get(centre) / dilute;
            dilute *= 2.0f;
        }
        noise *= 1.0f - ((centre - Vector2(0.5f, 0.5f)).magnitude() * 1.5f);
        i->data().sea = noise < waterRatio;
    }
    RemoveLakes(graph);
    maxHeight = ComputeDistanceTo(graph, DistanceToSea());
    ComputeDistanceTo(graph, DistanceToLand());
    
    setZValues(graph, maxHeight, maxZ);
    //setDown(graph);
    //calculateFlow(graph);
    //generateRivers(6, 1);
    //computeNormals();
}

void Continent::draw(Raster &raster) {
	TriangulatedTerrainGraph firstPass(graph);
	firstPass.smooth();
	firstPass.erode(8, 0.02f);
	TriangulatedTerrainGraph tri(firstPass);
	//tri.smooth();
	tri.erode(8, 0.02f);
	Rivers::Edges riverEdges;
	tri.findRivers(riverEdges);
	Mesh mesh;
	tri.toMesh(mesh);
	//mesh.smooth();
	//mesh.calculateNormals();
	ErosianMap grid(1024, 1024, 0.5f);
	Vector3 sun(0.0f, 0.5f, 1.0f);
	sun.normalize();
	mesh.rasterize(grid);
	/*grid.erode(10000000, 666);*/
	/*grid.raiseCliffs(32);
	tri.copyBackZValues(grid*/
	//Rivers rivers(riverEdges, 2, 0.05f);
	//Rivers::Meshes riverMeshes;
	//rivers.getMeshesAndCarveRiverBeds(grid, riverMeshes);
	/*grid.smooth();*/
	Grid<Vector3> normals(1024, 1024);
	grid.calculateNormals(normals);
	for (size_t y = 0; y != grid.height(); ++y) {
		for (size_t x = 0; x != grid.width(); ++x) {
			const Vector3 &vec = grid(x, y);
			const Vector3 &norm = normals(x, y);
			float shadow = std::max(0.0f, (norm * -1.0f).dot(sun));
			if (vec.z >= 0.0f) {
				raster(x, y) = (static_cast<uint32_t>(shadow * 0x99) << 8) | static_cast<uint32_t>((shadow * vec.z / maxZ) * 255) ;
			}
			else {
				raster(x, y) = static_cast<uint32_t>(shadow * 255) << 16 | 0x00006600;
			}
		}
	}
	//raster.draw(mesh, 0xffffffff);
}
