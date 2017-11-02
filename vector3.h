//
//  vector3.h
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef vector3_h
#define vector3_h

namespace worldmaker{
    struct Vector3{
        Vector3(float x, float y, float z) : x(x), y(y), z(z){}
        
        Vector3() {}

        float x, y, z;
        
        Vector3 operator-(const Vector3 &other) const{
            return Vector3(x - other.x, y - other.y, z - other.z);
        }
        
        float squareMagnitude() const{
            return (x * x) + (y * y) + (z * z);
        }
        
        float magnitude() const{
            return sqrtf(squareMagnitude());
        }
        
        Vector3 normalized() const{
            float mag = magnitude();
            return Vector3(x / mag, y / mag, z / mag);
        }
    };
}

#endif /* vector3_h */
