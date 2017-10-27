//
//  triangle.h
//  World Maker
//
//  Created by Jerome Johnson on 11/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef triangle_h
#define triangle_h

#include "circle.h"
#include <limits>
#include <algorithm>

namespace worldmaker{
    class Triangle{
    public:
        Vector2 vertices[3];
        
        void sortedVertices(Vector2 *vertices) const{
            std::copy(this->vertices, this->vertices + 3, vertices);
            std::sort(vertices, vertices + 3);
        }
    private:
        
        Vector2 collapsedCentre() const{
            Vector2 verts[3];
            sortedVertices(verts);
            return verts[0] + (verts[2] - verts[0]) * 0.5f;
        }
    public:
        Triangle(){}
        
        Triangle (Vector2 a, Vector2 b, Vector2 c) {
            vertices[0] = a;
            vertices[1] = b;
            vertices[2] = c;
        }
        
        void flipRotation (){
            Vector2 swap(vertices[0]);
            vertices[0] = vertices[2];
            vertices[2] = swap;
        }
        
        void getEdges (Edge *edges) const{
            edges[0](vertices[0], vertices[1]);
            edges[1](vertices[1], vertices[2]);
            edges[2](vertices[2], vertices[0]);
        }
        
        /*void sortVertices (){
            std::sort(vertices, vertices + 3);
        }*/
        
        bool isClockwise() const{
            Vector2 aToB(vertices[1] - vertices[0]);
            Vector2 bToC(vertices[2] - vertices[1]);
            return (aToB.x * bToC.y - aToB.y * bToC.x) > 0.0f;
        }
        
        bool isShit() const{
            return vertices[0] == vertices[1] || vertices[0] == vertices[2] || vertices[2] == vertices[1];
        }
        
        bool hasVertex(const Vector2 &point) const{
            for (int i = 0; i != 3; ++i){
                if (vertices[i] == point){
                    return true;
                }
            }
            return false;
        }
        
        BoundingRectangle boundingRectangle() const{
            return BoundingRectangle(vertices, vertices + 3);
        }
        
        Vector2 findCircumcircleCentre () const{
            //we need this to produce the same result regardless of the order of the vertices
            Vector2 sorted[3];
            sortedVertices(sorted);
            Edge la(sorted[0], sorted[1]), lb(sorted[0], sorted[2]);
            Vector2 intersection;
            Vector2 midA = la.midPoint(), midB = lb.midPoint();
            if ((Edge (midA, midA + la.normal()).intersection (Edge (midB, midB + lb.normal()), intersection))) {
                return intersection;
            } else {
                return collapsedCentre ();
            }
        }
        
        Vector2 findCentroid() const{
            //we need this to produce the same result regardless of the order of the vertices
            Vector2 sorted[3];
            sortedVertices(sorted);
            Edge la(sorted[0], sorted[1]), lb(sorted[0], sorted[2]);
            Vector2 midA = la.midPoint(), midB = lb.midPoint();
            Vector2 intersection;
            if (Edge(midA, sorted[2]).intersection (Edge(midB, sorted[1]), intersection)) {
                return intersection;
            } else {
                return collapsedCentre ();
            }
        }
        
        Circle circumcircle() const{
            Vector2 centre = findCircumcircleCentre ();
            return Circle (centre, (vertices[0] - centre).magnitude());
        }
        
        bool operator == (const Triangle &other) const{
            return vertices[0] == other.vertices[0] && vertices[1] == other.vertices[1] && vertices[2] == other.vertices[2];
        }
        
        bool operator != (const Triangle &other) const{
            return vertices[0] != other.vertices[0] && vertices[1] != other.vertices[1] && vertices[2] != other.vertices[2];
        }
        
        bool shares (const Edge &edge) const
        {
            Edge edges[3];
            getEdges(edges);
            for (int i = 0; i != 3; ++i){
                if (edges[i] == edge){
                    return true;
                }
            }
            return false;
        }
        
        bool intersects(const BoundingRectangle &rect) const{
            Edge edges[3];
            getEdges(edges);
            for (int i = 0; i != 3; ++i){
                if (rect.intersects(edges[i])){
                    return true;
                }
            }
            return false;
        }
        
        bool contains(const Vector2 &point) const{
            BoundingRectangle rect(boundingRectangle());
            if (!rect.contains(point)){
                return false;
            }
            else{
                Edge testEdge(rect.externalPoint(), point);
                Edge edges[3];
                getEdges(edges);
                int count = 0;
                for (int i = 0; i != 3; ++i){
                    if (testEdge.intersects(edges[i])){
                        ++count;
                    }
                }
                return (count & 1) == 1;
            }
        }
        
        friend std::ostream &operator<<(std::ostream &out, const Triangle &triangle){
            return out << triangle.vertices[0] << " <- " << triangle.vertices[1] << " <- " << triangle.vertices[2];
        }
    };
}

#endif /* triangle_h */
