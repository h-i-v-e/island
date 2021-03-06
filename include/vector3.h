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
#include <functional>
#include "vector2.h"

namespace motu{
    struct Vector3{
        Vector3(float x, float y, float z) : x(x), y(y), z(z){}
        
        Vector3() {}

        float x, y, z;

		Vector2 &asVector2() {
			return *reinterpret_cast<Vector2*>(this);
		}

		const Vector2 &asVector2() const {
			return *reinterpret_cast<const Vector2*>(this);
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

		Vector3 &operator/=(float f) {
			x /= f;
			y /= f;
			z /= f;
			return *this;
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
            return *this / magnitude();
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
            return *this /= magnitude();
        }
        
        bool operator < (const Vector3 &other) const{
            if (x < other.x){
                return true;
            }
			if (x > other.x) {
				return false;
			}
            if (y < other.y){
                return true;
            }
			if (y > other.y) {
				return false;
			}
            return z < other.z;
        }
        
        bool operator == (const Vector3 &other) const{
            return x == other.x && y == other.y && z == other.z;
        }

		bool operator != (const Vector3 &other) const {
			return x != other.x || y != other.y || z != other.z;
		}

		constexpr bool hasX() const {
			return x > FLT_EPSILON || x < -FLT_EPSILON;
		}

		constexpr bool hasY() const {
			return y > FLT_EPSILON || y < -FLT_EPSILON;
		}

		constexpr bool hasZ() const {
			return z > FLT_EPSILON || z < -FLT_EPSILON;
		}

		static Vector3 unitX() {
			return Vector3(1.0f, 0.0f, 0.0f);
		}

		static Vector3 unitY() {
			return Vector3(0.0f, 1.0f, 0.0f);
		}

		static Vector3 unitZ() {
			return Vector3(0.0f, 0.0f, 1.0f);
		}
        
        static Vector3 zero(){
            return Vector3(0.0f, 0.0f, 0.0f);
        }

		friend std::ostream &operator<<(std::ostream &out, const Vector3 &vec) {
			return out << "Vector3(" << vec.x << ", " << vec.y << ", " << vec.z << ')';
		}

		size_t hash() const {
			return (hashFloat(x) * 7) ^ (hashFloat(y) * 11) ^ (hashFloat(z) * 3);
		}
    };
}

#endif /* vector3_h */
