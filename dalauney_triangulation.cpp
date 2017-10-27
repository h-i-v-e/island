//
//  dalauney_triangulation.cpp
//  World Maker
//
//  Created by Jerome Johnson on 14/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "dalauney_triangulation.h"
#include "brtree.h"
#include "triangle.h"
#include <iostream>

using namespace worldmaker;

namespace{
    
    struct TriangleWithCircumcircle{
        Triangle triangle;
        Circle circumcircle;
        
        TriangleWithCircumcircle(){}
            
        TriangleWithCircumcircle(const Triangle &triangle) : triangle(triangle), circumcircle(triangle.circumcircle()){}
        
        template <class Itr>
        void findUnsharedEdges(Itr first, Itr last, std::vector<Edge> &output){
            Edge myEdges[3];
            triangle.getEdges(myEdges);
            for (int i = 0; i != 3; ++i){
                bool hasEdge = false;
                for (Itr j = first; j != last; ++j){
                    if (*j == *this){
                        continue;
                    }
                    if (j->triangle.shares(myEdges[i])){
                        hasEdge = true;
                        break;
                    }
                }
                if (!hasEdge){
                    output.push_back(myEdges[i]);
                }
            }
        }
        
        bool operator==(const TriangleWithCircumcircle &other) const{
            return triangle == other.triangle;
        }
    };
    
    typedef BRTree<TriangleWithCircumcircle> RTree;
    
    inline Triangle CreateBoundingTriangle(const std::vector<Vector2> &points){
        BoundingRectangle bounds(points.begin(), points.end());
        bounds.topLeft += Vector2 (-1.0f, -1.0f);
        bounds.bottomRight += Vector2 (1.0f, 1.0f);
        float width = bounds.width(), height = bounds.height();
        Vector2 apex(bounds.topLeft.x + width * 0.5f, bounds.topLeft.y - width * 0.6667f);
        float spread = height * 0.6667f;
        Vector2 left(bounds.topLeft.x - spread, bounds.bottomRight.y);
        Vector2 right(bounds.bottomRight.x + spread, bounds.bottomRight.y);
        return Triangle (left, apex, right);
    }

    inline bool OfSuperTriangle(const Triangle &super, const Triangle &triangle){
        for (int i = 0; i != 3; ++i){
            for (int j = 0; j != 3; ++j){
                if (super.vertices[i] == triangle.vertices[j]){
                    return true;
                }
            }
        }
        return false;
    }
    
    inline bool AddBadTriangles(const Vector2 &point, std::vector<TriangleWithCircumcircle> &badTriangles, const RTree &rTree, RTree::Values &buf){
        auto pair = rTree.containing(point, buf);
        while (pair.first != pair.second){
            if (pair.first->triangle.hasVertex(point)){
                return false;
            }
            if (pair.first->circumcircle.contains (point)) {
                badTriangles.push_back(*pair.first);
            }
            ++pair.first;
        }
        return true;
    }
}
    
DalauneyTriangulation::DalauneyTriangulation(const std::vector<Vector2> &points){
    size_t expectedTriangles = points.size() << 1;
    pool = new HalfEdgePool<HalfEdge<>>(expectedTriangles * 3);
    RTree rTree(expectedTriangles);
    TriangleWithCircumcircle super(CreateBoundingTriangle(points));
    rTree.add(super.circumcircle.boundingRectangle(), super);
    std::vector<TriangleWithCircumcircle> badTriangles;
    RTree::Values searchBuffer;
    searchBuffer.reserve(expectedTriangles);
    std::vector<Edge> poly;
    for (auto i = points.begin(); i != points.end(); ++i){
        if (!AddBadTriangles(*i, badTriangles, rTree, searchBuffer)){
            continue;
        }
        for (auto i = badTriangles.begin(); i != badTriangles.end(); ++i){
            i->findUnsharedEdges(badTriangles.begin(), badTriangles.end(), poly);
        }
        for (auto bad : badTriangles){
            rTree.remove(bad.circumcircle.boundingRectangle(), bad);
        }
        for (auto edge : poly){
            TriangleWithCircumcircle tri(Triangle(edge.endA, edge.endB, *i));
            if (!tri.triangle.isClockwise()){
                tri.triangle.flipRotation();
            }
            rTree.add(tri.circumcircle.boundingRectangle(), tri);
        }
        poly.clear();
        badTriangles.clear();
    }
    auto pair = rTree.all(searchBuffer);
    HalfEdge<>::Builder builder;
    while (pair.first != pair.second){
        if (!OfSuperTriangle(super.triangle, pair.first->triangle)){
            builder.add(HalfEdge<>::fromPolygon(pair.first->triangle.vertices, pair.first->triangle.vertices + 3, *pool));
        }
        ++pair.first;
    }
    builder.construct();
}

DalauneyTriangulation::~DalauneyTriangulation(){
    delete pool;
}
