//
//  bounding_rectangle.h
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef bounding_rectangle_h
#define bounding_rectangle_h

#include <limits>
#include "edge.h"
#include <functional>

namespace motu{
    
    struct BoundingRectangle{
		typedef Vector2 VectorType;

        Vector2 topLeft, bottomRight;

        BoundingRectangle(Vector2 topLeft, Vector2 bottomRight) : topLeft(topLeft), bottomRight(bottomRight){}
        
        BoundingRectangle(float top, float left, float bottom, float right) : topLeft(top, left), bottomRight(bottom, right){}
        
        BoundingRectangle(Vector2 vec) : topLeft(vec), bottomRight(vec){}
        
        BoundingRectangle(const BoundingRectangle &other) : topLeft(other.topLeft), bottomRight(other.bottomRight){}
        
        BoundingRectangle() {
			clear();
		}
        
        void clear(){
            topLeft(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
            bottomRight(std::numeric_limits<float>::min(), std::numeric_limits<float>::min());
        }
        
        void add(const Vector2 &point){
            if (point.x < topLeft.x){
                topLeft.x = point.x;
            }
            if (point.x > bottomRight.x){
                bottomRight.x = point.x;
            }
            if (point.y < topLeft.y){
                topLeft.y = point.y;
            }
            if (point.y > bottomRight.y){
                bottomRight.y = point.y;
            }
        }
        
        template <class Itr>
        BoundingRectangle(Itr first, Itr last){
            clear();
            while (first != last){
                add(*first++);
            }
        }
        
        bool operator == (const BoundingRectangle &other) const{
            return topLeft == other.topLeft && bottomRight == other.bottomRight;
        }
        
        bool operator != (const BoundingRectangle &other) const{
            return topLeft != other.topLeft && bottomRight != other.bottomRight;
        }
        
        bool contains(const Vector2 &point) const{
            return Edge::between(topLeft.x, bottomRight.x, point.x) && Edge::between(topLeft.y, bottomRight.y, point.y);
        }
        
        void getVertices(Vector2 *vertices) const{
            vertices[0] = topLeft;
            vertices[1](bottomRight.x, topLeft.y);
            vertices[2] = bottomRight;
            vertices[3](topLeft.x, bottomRight.y);
        }
        
        bool empty() const{
            return topLeft.x == std::numeric_limits<float>::max() &&
            topLeft.y == std::numeric_limits<float>::max() &&
            bottomRight.x == std::numeric_limits<float>::min() &&
            bottomRight.y == std::numeric_limits<float>::min();
        }
        
        void operator+= (const BoundingRectangle &other){
            add(other.topLeft);
            add(other.bottomRight);
        }
        
        BoundingRectangle operator+(const BoundingRectangle &other) const{
            BoundingRectangle o(*this);
            o += other;
            return o;
        }
        
        void getEdges(Edge *edges) const{
            edges[0](topLeft.x, topLeft.y, bottomRight.x, topLeft.y);
            edges[1](bottomRight.x, topLeft.y, bottomRight.x, bottomRight.y);
            edges[2](bottomRight.x, bottomRight.y, topLeft.x, bottomRight.y);
            edges[3](topLeft.x, bottomRight.y, topLeft.x, topLeft.y);
        }
        
        bool intersects(const BoundingRectangle &other) const{
            Edge edges[4];
            getEdges(edges);
            for (int i = 0; i != 4; ++i){
                if (other.contains(edges[i].endA) || other.contains(edges[i].endB)){
                    return true;
                }
            }
            other.getEdges(edges);
            for (int i = 0; i != 4; ++i){
                if (contains(edges[i].endA) || contains(edges[i].endB)){
                    return true;
                }
            }
            return false;
        }
        
        float width() const{
            return bottomRight.x - topLeft.x;
        }
        
        float height() const{
            return bottomRight.y - topLeft.y;
        }
        
        Vector2 centre() const{
            return Vector2 (topLeft.x + width() * 0.5f, topLeft.y + height() * 0.5f);
        }
        
        Vector2 externalPoint() const{
            return topLeft + (topLeft - centre()) * 2.0f;
        }
        
        float area() const{
            return width() * height();
        }
        
        bool intersection(const BoundingRectangle &other, BoundingRectangle &output) const{
            output.clear();
            Vector2 vertices[4];
            getVertices(vertices);
            for (int i = 0; i != 4; ++i){
                if (other.contains(vertices[i])){
                    output.add(vertices[i]);
                }
            }
            other.getVertices(vertices);
            for (int i = 0; i != 4; ++i){
                if (contains(vertices[i])){
                    output.add(vertices[i]);
                }
            }
            return output.area() > 0.0f;
        }
        
        bool intersects(const Edge &edge) const{
            if (contains(edge.endA) || contains(edge.endB)){
                return true;
            }
            else{
                Edge edges[4];
                getEdges(edges);
                for (int i = 0; i != 4; ++i){
                    if (edges[i].intersects(edge)){
                        return true;
                    }
                }
                return false;
            }
        }
        
        friend std::ostream &operator<<(std::ostream &o, const BoundingRectangle &rect){
            return o << "BoundingRectangle(" << rect.topLeft << ", " << rect.bottomRight << ')';
        }
        
    };
}


#endif /* bounding_rectangle_h */
