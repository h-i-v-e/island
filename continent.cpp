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
#include "triangulated_voronoi.h"
#include "erosian_map.h"

using namespace worldmaker;
using namespace std;

namespace{
    //std::set<Continent::Face*> riverFaces;
    Random randomness;
    
    struct ErosianFill{
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
    };
    
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
        void GetCentroids(const TerrainGraph &graph, std::vector<Vector2> &out){
        out.clear();
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (i->face().halfEdge() == &(*i)){
                out.push_back(i->face().calculateCentroid());
            }
        }
    }

    
    const TerrainGraph::HalfEdge &FindPerimeter(const TerrainGraph &graph){
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (i->onPerimeter()){
                return *i;
            }
        }
        return *graph.halfEdges().begin();
    }
    
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
    
    Vector3 vertexToVector3(const TerrainGraph::Vertex &vertex){
        return Vector3(vertex.position().x, vertex.position().y, vertex.data().z);
    }
    
    float computeAverageZ(const TerrainGraph::Face &face){
        float totalZ = 0.0;
        int total = 0;
        for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
            totalZ += i->vertex().data().z;
            ++total;
        }
        return totalZ / total;
        /*float minZ = numeric_limits<float>::max(), maxZ = numeric_limits<float>::min();
        for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
            float z = i->vertex().data().z;
            if (z < minZ){
                minZ = z;
            }
            if (z > maxZ){
                maxZ = z;
            }
        }
        return minZ + (randomness.get() * (maxZ - minZ));*/
    }
    
    typedef std::vector<Triangle3> Triangulation;
    
    Vector3 fromVertex(const TerrainGraph::Vertex &vertex){
        const Vector2 &vec = vertex.position();
        return Vector3(vec.x, vec.y, vertex.data().z);
    }
    
    void RemoveLakes(TerrainGraph &graph){
        std::set<TerrainGraph::Face> sea;
        MapSea(FindPerimeter(graph), sea);
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
            dt.target(i->data()) = numeric_limits<int>::max();
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
    }
    
    void setDown(TerrainGraph::Vertex &vertex){
        if (vertex.data().z == 0.0f){
            vertex.data().down = nullptr;
            return;
        }
        Vector3 pos(vertex.position().x, vertex.position().y, vertex.data().z);
        float steepest = 1.1f;
        TerrainGraph::Vertex *winner = nullptr;
        auto itr = vertex.inbound();
        for (auto j = itr.begin(); j != itr.end(); ++j){
            if (j->face().data().sea){
                winner = nullptr;
                break;
            }
            if (j->next->vertex().data().down == &vertex){
                continue;
            }
            Vector3 other(j->next->vertex().position().x, j->next->vertex().position().y, j->next->vertex().data().z);
            Vector3 result(other - pos);
            result = result.normalized();
            if (result.z < steepest){
                steepest = result.z;
                winner = &j->pair->vertex();
            }
            else if (result.z == steepest && (j->next->vertex().position() - vertex.position()).magnitude() < (j->next->vertex().position() - winner->position()).magnitude()){
                steepest = result.z;
                winner = &j->pair->vertex();
            }
        }
        vertex.data().down = winner && steepest <= 0.0f ? winner : nullptr;
    }
    
    void setDown(TerrainGraph &graph){
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            setDown(*i);
        }
    }
    
    void calculateFlow(TerrainGraph &graph){
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            i->data().flow = 0;
        }
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            for (auto *down = i->data().down; down; down = down->data().down){
                ++down->data().flow;
            }
        }
    }
    
    inline bool isCoastal(const TerrainGraph::HalfEdge &edge){
        return edge.pair && edge.face().data().sea != edge.pair->face().data().sea;
    }
    
    typedef std::set<const TerrainGraph::HalfEdge *> VisitedSet;
    
    inline TerrainGraph::HalfEdge *findNextCoast(const VisitedSet &visited, const TerrainGraph::HalfEdge &edge){
        for (auto j = edge.next->vertex().inbound().begin(); j != edge.next->vertex().inbound().end(); ++j){
            if (isCoastal(*j) && visited.find(&*j) == visited.end()){
                return &*j;
            }
        }
        return nullptr;
    }
    
    void mapCoastlines(TerrainGraph &graph, std::vector<std::vector<TerrainGraph::HalfEdge*>> &coastlines){
        VisitedSet visited;
        for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
            if (isCoastal(*i) && visited.find(&*i) == visited.end()){
                coastlines.emplace_back();
                TerrainGraph::HalfEdge *start = &*i;
                do {
                    coastlines.back().push_back(start);
                    visited.insert(start);
                    visited.insert(start->pair);
                    if (!(start = findNextCoast(visited, *start))){
                        break;
                    }
                } while (start != &*i);
            }
        }
    }
    
    
    void computeDisplacementBlobby(Vector2 &out, const TerrainGraph::HalfEdge &a, TerrainGraph::HalfEdge &b, const TerrainGraph::HalfEdge &c){
        float cos = (a.edge().direction() - c.edge().direction()).normalized().dot(b.edge().normal().normalized());
        out = b.edge().normal() * cos * ((randomness.get() * 0.0625f) + 0.0625f);
    }
    
    void computeDisplacement(Vector2 &out, const TerrainGraph::HalfEdge &a, TerrainGraph::HalfEdge &b, const TerrainGraph::HalfEdge &c){
        computeDisplacementBlobby(out, a, b, c);
    }
    
    /*void tesselateCoastline(TerrainGraph &graph, TerrainGraph::HalfEdges &coast){
        std::vector<Vector2> displacements;
        displacements.assign(coast.size(), Vector2());
        size_t last = coast.size() - 1;
        for (size_t i = 1; i < last; ++i){
            computeDisplacement(displacements[i], *coast[i - 1], *coast[i], *coast[i + 1]);
        }
        computeDisplacement(displacements[last], *coast[last - 1], *coast[last], **coast.begin());
        computeDisplacement(displacements[0], *coast[last], **coast.begin(), *coast[1]);
        
        
        coast.reserve(coast.size() << 1);
        Continent::HalfEdges temp;
        temp.reserve(coast.size());
        copy(coast.begin(), coast.end(), back_inserter(temp));
        coast.clear();
        for (int i = 0; i != temp.size(); ++i){
            coast.push_back(temp[i]);
            Continent::Vertex *insert = graph.vertices().allocate();
            insert->data().z = 0.0f;
            temp[i]->split(insert, graph.halfEdges());
            coast.push_back(temp[i]->next);
            insert->position() += displacements[i];
        }
        
    }*/
    
    /*void tesselateCoastline(Continent::Graph &graph, Continent::Coastlines &coast){
        for (auto i = coast.begin(); i != coast.end(); ++i){
            tesselateCoastline(graph, *i);
        }
    }*/
    
    int computeMaxFlow(TerrainGraph &graph){
        int maxFlow = 0;
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            int flow = i->data().flow;
            if (flow > maxFlow){
                maxFlow = flow;
            }
        }
        return maxFlow;
    }
    
    void carveRavines(TerrainGraph &graph, float erosianLevel){
        float maxFlow = computeMaxFlow(graph);
        for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
            float flow = i->data().flow;
            if (flow > 0.0f){
                i->data().z -= ((flow / maxFlow) * i->data().z * erosianLevel);
                i->data().z = max<float>(i->data().z, 0.0f);
            }
        }
    }
    
    /*typedef std::vector<Triangle3> Triangles;
    
    void getTriangles(const Continent::Face &face, Triangles &triangles){
        Vector2 centre(face.calculateCentroid());
        Vector3 centre3(centre.x, centre.y, computeAverageZ(face));
        auto i = face.halfEdges().begin();
        auto last = i;
        for (++i; i != face.halfEdges().end(); ++i){
            triangles.emplace_back(fromVertex(last->vertex()), fromVertex(i->next->vertex()), centre3);
        }
        triangles.emplace_back(fromVertex(last->vertex()), fromVertex(face.halfEdges().begin()->next->vertex()), centre3);
    }*/
    
    Vector3 computeNormal(const TerrainGraph::Vertex &vertex){
        Vector3 total(0.0f, 0.0f, 0.0f);
        for (auto i = vertex.inbound().begin(); i != vertex.inbound().end(); ++i){
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
    
    struct SolidColourFill{
        uint32_t colour;
        
        SolidColourFill(uint32_t colour) : colour(colour){}
        
        uint32_t operator()(const Raster::Vector3WithNormal &vals) const{
            return colour;
        }
    };
    
    Vector2 averageAngle(const TerrainGraph::Vertex &vert, const TerrainGraph::HalfEdge &ignore){
        Vector2 total(0.0f, 0.0f);
        for (auto i = vert.inbound().begin(); i != vert.inbound().end(); ++i){
            if (&*i == &ignore){
                continue;
            }
            total += i->edge().direction().normalized();
        }
        return total.normalized();
    }
    
    bool computeDisplacementDirection(const TerrainGraph::Vertex &vert, Vector2 &output) {
        auto i = TerrainGraph::HalfEdge::findHalfEdgeConnecting(vert.inbound().begin(), vert.inbound().end(), *vert.data().down);
        if (i == vert.inbound().end()){
            return false;
        }
        Vector2 in(averageAngle(vert, *i));
        if (!vert.data().down->data().down){
            return false;
        }
        auto j = TerrainGraph::HalfEdge::findHalfEdgeConnecting(vert.data().down->inbound().begin(), vert.data().down->inbound().end(), *vert.data().down->data().down);
        Vector2 out((j == vert.data().down->inbound().end()) ? in : averageAngle(*vert.data().down, *j));
        Vector2 normal(i->edge().normal());
        output = normal * (in + out).dot(normal);
        return true;
    }
}

void Continent::computeNormals(){
    /*std::vector<Vector3> neighbours;
    for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
        neighbours.clear();
        for (auto j = i->inbound().begin(); j != i->inbound().end(); ++j){
            const Vector2 &vert = j->next->vertex().position();
            neighbours.emplace_back(vert.x, vert.y, j->next->vertex().data().z);
        }
        i->data().normal = Triangle3::computeNormal(Vector3(i->position().x, i->position().y, i->data().z), neighbours.begin(), neighbours.end());
    }*/
    for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
        i->data().normal = computeNormal(*i);
    }
}

/*void Continent::tesselate(HalfEdges &edges){
    edges.reserve(edges.size() << 1);
    HalfEdges temp;
    copy(edges.begin(), edges.end(), back_inserter(temp));
    edges.clear();
    for (auto i = temp.begin(); i != temp.end(); ++i){
        Vector2 displacementDirection;
        if (!computeDisplacementDirection((*i)->vertex(), displacementDirection)){
            displacementDirection = (*i)->edge().normal() * (randomness.get() - 0.5f);
        }
        edges.push_back(*i);
        Vertex *insert = graph.vertices().allocate();
        insert->data().z = ((*i)->vertex().data().z + (*i)->next->vertex().data().z) * 0.5f;
        insert->data().flow = (*i)->vertex().data().flow;
        insert->data().down = (*i)->vertex().data().down;
        (*i)->split(insert, graph.halfEdges());
        (*i)->vertex().data().down = insert;
        insert->position() += displacementDirection.normalized() * (*i)->edge().direction().magnitude() * ((randomness.get() * 0.25f) + 0.25f);
        edges.push_back((*i)->next);
    }
}*/

/*void Continent::generateRivers(int flowThreshold, int tesselations){
    TerrainGraph::HalfEdges ravines;
    for (auto i = graph.vertices().begin(); i != graph.vertices().end(); ++i){
        if (i->data().down){
            auto down = Continent::HalfEdge::findHalfEdgeConnecting(i->inbound().begin(), i->inbound().end(), *i->data().down);
            if (down != i->inbound().end()){
                if (i->data().flow > flowThreshold){
                    rivers.push_back(&*down);
                }
                else{
                    ravines.push_back(&*down);
                }
            }
        }
    }
    mapCoastlines(graph, mCoastlines);
    tesselate(rivers);
    //tesselate(ravines);
    //carveRavines(graph, 1.0f);
    tesselateCoastline(graph, mCoastlines);
}*/

/*void Continent::generateTiles(int relaxations){
    std::vector<Vector2> points;
    points.reserve(numTiles);
    for (int i = 0; i != numTiles; ++i){
        points.push_back(randomness.vector2());
    }
    for (int i = 0;;){
        DalauneyTriangulation<> triangulation(points);
        graph.generate(triangulation.halfEdges().begin(), triangulation.halfEdges().end());
        if (++i == relaxations){
            break;
        }
        GetCentroids(graph, points);
    }
}*/

void Continent::generateSeasAndLakes(float waterRatio){
    NoiseLayer layers[4];
    float strength = 2.0f;
    for (int i = 0; i != 4; ++i){
        layers[i] = NoiseLayer(randomness.vector2(), strength);
        strength *= 4.0f;
    }
    for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
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

void Continent::draw(Raster &raster){
    SolidColourFill seaFill(0x00ff7700);
    Vector3 sun(0.0f, 1.0f, 1.0f);
    sun.normalize();
    ErosianFill erosian(raster.width(), raster.height(), 0.5f);
    typedef TriangulatedVoronoi<TerrainGraph> UnterTri;
    UnterTri unter(graph);
    for (int i = 0; i != 2; ++i){
        unter.smooth();
    }
    unter.computeNormals();
    Triangle3WithNormals tri;
    for (auto i = unter.faces().begin(); i != unter.faces().end(); ++i){
        raster.fillTriangle(UnterTri::copyTo(*i, tri), erosian);
    }
    erosian.erosian.erode(10000000, 25918);
    erosian.erosian.smooth();
    Grid<Vector3> normals(raster.width(), raster.height());
    erosian.erosian.calculateNormals(normals);
    float invZ = 1.0 / maxZ;
    for (int y = 0; y != normals.height(); ++y){
        for (int x = 0; x != normals.width(); ++x){
            Vector3 normal(normals(x, y));
            float z = erosian.erosian(x, y).z * invZ;
            uint32_t nInt = std::max<int>(std::min<int>(sun.dot(normal * -1.0f) * 255, 255), 0);
            if (z < 0.0f){
                float blend = std::max(1.0f + (z * 20.0f), 0.0f) * 0.75f;
                uint32_t red = (nInt >> 1) * blend;
                uint32_t green = (static_cast<uint32_t>(nInt * blend) + 0x99) >> 1;
                raster(x, y) = red | (green << 8) | 0x00990000;
            }
            else{
                uint32_t zInt = std::min<int>(z * 255, 255);
                uint32_t red = std::min<uint32_t>((zInt + nInt) >> 1, 255);
                uint32_t green = std::min<uint32_t>((0x99 + nInt) >> 1, 255);
                uint32_t blue = std::min<uint32_t>(((zInt >> 1) + nInt) >> 1, 255);
                raster(x, y) = red | (green << 8) | (blue << 16);
            }
        }
    }
    /*ErosianMap::Rivers rivers;
    erosian.erosian.trackRivers(rivers, 1024);
    for (auto i = rivers.begin(); i != rivers.end(); ++i){
        raster(i->first, i->second) = 0x00ff0000;
    }*/
    /*for (auto i = ret.halfEdges().begin(); i != ret.halfEdges().end(); ++i){
        raster.draw(i->edge(), 0xffffffff);
    }*/
    /*for (auto i = triangulation.vertices().begin(); i != triangulation.vertices().end(); ++i){
        if (i->data().down){
            raster.draw(Edge(i->position(), i->data().down->position()), 0xffffffff);
        }
    }*/
    /*for (auto i = triangulation.halfEdges().begin(); i != triangulation.halfEdges().end(); ++i){
        raster.draw(i->edge(), 0x99999999);
    }*/
    /*for (auto i = graph.halfEdges().begin(); i != graph.halfEdges().end(); ++i){
        raster.draw(i->edge(), 0x00000000);
    }*/
    
    /*for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        for (auto j : i->halfEdges()){
            raster.draw(j.edge(), 0x66666666, 4);
            auto down = HalfEdge::findHalfEdgeConnecting(j.vertex().inbound().begin(), j.vertex().inbound().end(), *j.vertex().data().down);
            if (down != j.vertex().inbound().end() && *down == j){
                raster.draw(j.edge(), 0x99999999, 4);
            }
        }
    }*/
    /*tesselateCoastline(graph, mCoastlines);
    for (auto i = mCoastlines.begin(); i != mCoastlines.end(); ++i){
        for (auto j = i->begin(); j != i->end(); ++j){
            raster.draw((*j)->edge(), 0xffffffff);
        }
    }*/
    /*for (auto i = graph.faces().begin(); i != graph.faces().end(); ++i){
        for (auto j : i->halfEdges()){
            if (j.pair && j.pair->face().data().sea != i->data().sea){
                raster.draw(j.edge(), 0x00009999, 4);
            }
            else{
                raster.draw(j.edge(), 0x66666666, 4);
            }
            auto down = HalfEdge::findHalfEdgeConnecting(j.vertex().inbound().begin(), j.vertex().inbound().end(), *j.vertex().data().down);
            if (down != j.vertex().inbound().end() && *down == j){
                raster.draw(j.edge(), 0x99999999, 4);
            }
        }
    }*/
    /*for (auto i = riverFaces.begin(); i != riverFaces.end(); ++i){
        raster.fill(**i, 0xffffffff);
    }*/
    /*for (auto i = rivers.begin(); i != rivers.end(); ++i){
        raster.draw((*i)->edge(), 0x00ff9999);
    }*/
}
