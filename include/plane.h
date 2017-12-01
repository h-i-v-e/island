//
//  plane.h
//  World Maker
//
//  Created by Jerome Johnson on 21/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef plane_h
#define plane_h

namespace motu{
    struct Plane{
        Vector3 point, normal;
        
        bool intersectionTime(const Vector3 &point, const Vector3 &direction, float &when) const{
            float nd = normal.dot(direction);
            if (nd == 0.0f){
                return false;
            }
            when = (this->point - point).dot(normal) / nd;
            return true;
        }
        
        float distanceTo(Vector3 point) const{
            return (this->point - point).dot(normal);
        }
        
        bool intersection(const Vector3 &point, const Vector3 &direction, Vector3 &result) const{
            float when;
            if (intersectionTime(point, direction, when)){
                result = point + (direction * when);
                return true;
            }
            return false;
        }
    };
}

#endif /* plane_h */
