//
//  circle.h
//  World Maker
//
//  Created by Jerome Johnson on 12/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef circle_h
#define circle_h

#include "bounding_rectangle.h"
#include <iostream>

namespace worldmaker{
    class Circle{
    private:
        Vector2 centre;
        float radius;
        
    public:
        Circle(){}
        
        Circle (const Vector2 &centre, float radius) : centre(centre), radius(radius){}
        
        Circle (const Vector2 &centre, const Vector2 &pointOnEdge) : centre(centre){
            radius = (pointOnEdge - centre).magnitude();
            if (radius < 0.0f) {
                radius = -radius;
            }
        }
        
        bool contains (const Vector2 &point) const{
            float squareRadius = radius * radius;
            float sqm = (centre - point).sqrMagnitude();
            return sqm >= -squareRadius && sqm <= squareRadius;
        }
        
        bool intersects(const Edge &edge) const{
            Vector2 contact;
            Edge (centre, centre + edge.normal()).intersection (edge, contact);
            return (contact - centre).sqrMagnitude() <= radius * radius;
        }
        
        BoundingRectangle boundingRectangle() const{
            return BoundingRectangle(centre.x - radius, centre.y - radius, centre.x + radius, centre.y + radius);
        }
        
        bool intersects(const BoundingRectangle &rect) const{
            Edge edges[4];
            rect.getEdges(edges);
            for (int i = 0; i != 4; ++i){
                if (intersects(edges[i])){
                    return true;
                }
            }
            return false;
        }
        
        friend std::ostream &operator<<(std::ostream &o, const Circle &circle){
            return o << "Centre: " << circle.centre << ", radius: " << circle.radius;
        }
    };
}

#endif /* circle_h */
