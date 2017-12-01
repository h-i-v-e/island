//
//  vector2.h
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef vector2_h
#define vector2_h

#include <cmath>
#include <iostream>

namespace motu {
    struct Vector2{
        Vector2(){}
        
        Vector2(float x, float y) : x(x), y(y){}
        
        Vector2 &zero(){
            x = y = 0.0f;
            return *this;
        }
        
        const Vector2 &operator()(float x, float y){
            this->x = x;
            this->y = y;
            return *this;
        }
        
        Vector2 operator+(const Vector2 &other) const{
            return Vector2(x + other.x, y + other.y);
        }
        
        Vector2 &operator+=(const Vector2 &other){
            x += other.x;
            y += other.y;
            return *this;
        }
        
        Vector2 operator-(const Vector2 &other) const{
            return Vector2(x - other.x, y - other.y);
        }
        
        Vector2 operator*(float f) const{
            return Vector2(x * f, y * f);
        }
        
        Vector2 &operator*=(float f) {
            x *= f;
            y *= f;
            return *this;
        }
        
        Vector2 operator/(float f) const{
            return Vector2(x / f, y / f);
        }
        
        Vector2 normal() const{
            return Vector2(y, -x);
        }
        
        float sqrMagnitude() const{
            return x * x + y * y;
        }
        
        float magnitude() const{
            return sqrtf(sqrMagnitude());
        }
        
        Vector2 normalized() const{
            float mag = magnitude();
            return Vector2(x / mag, y / mag);
        }
        
        constexpr bool operator == (const Vector2 &other) const{
            return x == other.x && y == other.y;
        }
        
        constexpr bool operator != (const Vector2 &other) const{
            return x != other.x && y != other.y;
        }
        
        bool operator < (const Vector2 &other) const{
            if (y < other.y){
                return true;
            }
            if (y > other.y){
                return false;
            }
            return x < other.x;
        }
        
        constexpr float dot(const Vector2 &other) const{
            return x * other.x + y * other.y;
        }

        float x, y;
        
        friend std::ostream &operator<<(std::ostream &o, const Vector2 &vec){
            return o << "Vector2(" << vec.x << ", " << vec.y << ')';
        }
    };
}

#endif /* vector2_h */
