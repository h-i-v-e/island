//
//  edge.h
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef edge_h
#define edge_h

#include "vector2.h"
#include "vector3.h"

namespace motu{
    
    struct Edge{
        Edge(){}
        
        Edge(const Vector2 &endA, const Vector2 &endB) : endA(endA), endB(endB){}
        
        Edge(float ax, float ay, float bx, float by) : endA(ax, ay), endB(bx, by) {}

        
        Edge &operator()(const Vector2 &endA, const Vector2 &endB){
            this->endA = endA;
            this->endB = endB;
            return *this;
        }
        
        Edge &operator()(float ax, float ay, float bx, float by){
            endA(ax, ay);
            endB(bx, by);
            return *this;
        }
        
        bool operator==(const Edge &other) const{
            return (endA == other.endA && endB == other.endB) || (endA == other.endB && endB == other.endA);
        }
        
        bool operator!=(const Edge &other) const{
            return !(*this == other);
        }
        
        Vector2 direction() const{
            return endB - endA;
        }
        
        float length() const{
            return direction().magnitude();
        }
        
        Vector2 perp() const{
            return Vector2(endA.y - endB.y, endB.x - endA.x);
        }
        
        Vector2 midPoint() const{
            return endA + ((endB - endA) * 0.5f);
        }
        
        Vector3 parametric() const{
            Vector2 direction = Edge::direction();
            return Vector3(-direction.y, direction.x, direction.y * endA.x - direction.x * endA.y);
        }
        
        bool parallel(const Edge &edge) const{
            return perp().dot(edge.direction()) == 0.0f;
        }
        
        bool intersectionTime(const Edge &edge, float &result) const{
            Vector2 n(perp());
            float d = n.dot(edge.direction());
            if (d == 0.0f){
                return false;
            }
            result = (endA - edge.endA).dot(n) / d;
            return true;
        }
        
        bool intersection(const Edge &other, Vector2 &result) const{
            float hitTime;
            if (intersectionTime(other, hitTime)){
                result = other.endA + (other.direction() * hitTime);
                return true;
            }
            return false;
        }
        
        static constexpr bool between (float a, float b, float c){
            return (c >= a && c <= b) || (c <= a && c >= b);
        }

		bool between(const Vector2 &vec) const{
			return between(endA.x, endB.x, vec.x) && between(endA.y, endB.y, vec.y);
		}

        bool intersects(const Edge &b, Vector2 &result) const{
			return intersection(b, result) && between(result) && b.between(result);
        }
        
        bool intersects(const Edge &b) const{
            Vector2 i;
            return intersects(b, i);
        }
        
        Vector2 endA, endB;
    };
}


#endif /* edge_h */
