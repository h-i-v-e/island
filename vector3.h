//
//  vector3.h
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef vector3_h
#define vector3_h

#include <cmath>
#include "vector2.h"

namespace worldmaker{
    struct Vector3{
        Vector3(float x, float y, float z) : x(x), y(y), z(z){}
        
        Vector3() {}

        float x, y, z;
        
        Vector2 toVector2() const{
            return Vector2(x, y);
        }
        
        Vector3 operator-(const Vector3 &other) const{
            return Vector3(x - other.x, y - other.y, z - other.z);
        }
        
        Vector3 &operator-=(const Vector3 &other) {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }
        
        Vector3 operator+(const Vector3 &other) const{
            return Vector3(x + other.x, y + other.y, z + other.z);
        }
        
        Vector3 operator/(float f) const{
            return Vector3(x / f, y / f, z / f);
        }
        
        Vector3 &operator+=(const Vector3 &other){
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }
        
        Vector3 cross(const Vector3 &other) const{
            return Vector3((y * other.z) - (z * other.y), (z * other.x) - (x * other.z), (x * other.y) - (other.x * y));
        }
        
        float dot(const Vector3 &other) const{
            return x * other.x + y * other.y + z * other.z;
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
        
        Vector3 operator * (float f) const{
            return Vector3(x * f, y * f, z * f);
        }
        
        Vector3 &operator *= (float f) {
            x *= x;
            y *= y;
            z *= z;
            return *this;
        }
        
        Vector3 &normalize(){
            float mag = magnitude();
            x /= mag;
            y /= mag;
            z /= mag;
            return *this;
        }
        
        static Vector3 zero(){
            return Vector3(0.0f, 0.0f, 0.0f);
        }
    };
}

#endif /* vector3_h */
