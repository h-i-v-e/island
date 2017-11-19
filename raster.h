//
//  raster.h
//  World Maker
//
//  Created by Jerome Johnson on 29/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef raster_h
#define raster_h

#include <cstdint>
#include "bounding_rectangle.h"
#include <limits>
#include <algorithm>
#include "triangle.h"
#include "spline.h"
#include "triangle3.h"

namespace worldmaker{
    
    class Raster{
    private:
        
        uint32_t *mData;
    public:
        Raster(uint32_t width, uint32_t height) : mData(new uint32_t[width * height + 2]){
            mData[0] = width;
            mData[1] = height;
        }
        
        ~Raster(){
            delete [] mData;
        }
        
        int width() const{
            return static_cast<int>(mData[0]);
        }
        
        int height() const{
            return static_cast<int>(mData[1]);
        }
        
        int length() const{
            return width() * height();
        }
        
        uint32_t *data(){
            return mData + 2;
        }
        
        uint32_t &operator()(int x, int y){
            return data()[(width() * y) + x];
        }
        
        uint32_t operator()(int x, int y) const{
            return data()[(width() * y) + x];
        }

        const uint32_t *data() const{
            return mData + 2;
        }
        
        void fill(uint32_t colour){
            for (uint32_t *i = mData + 2, *j = mData + (mData[0] * mData[1] + 2); i != j; ++i){
                *i = colour;
            }
        }
        
        static float getZMul(const Spline &intersected, float x){
            return (((x - intersected.endA.x) / (intersected.endB.x - intersected.endA.x)) * (intersected.endB.z - intersected.endA.z)) + intersected.endA.z;
        }
        
        struct Vector3WithNormal : public Vector3{
            Vector3 normal;
            
            Vector3WithNormal(){}
            
            Vector3WithNormal(const Vector3 &vect, const Vector3 &normal) : Vector3(vect), normal(normal){}
            
            Vector3WithNormal operator-(const Vector3WithNormal &other) const{
                return Vector3WithNormal(Vector3::operator-(other), normal - other.normal);
            }
            
            Vector3WithNormal operator/(float f) const{
                return Vector3WithNormal(Vector3::operator/(f), normal / f);
            }
            
            Vector3WithNormal &operator += (const Vector3WithNormal &other){
                Vector3::operator+=(other);
                normal += other.normal;
                return *this;
            }
            
            static Vector3WithNormal zero(){
                return  Vector3WithNormal(Vector3::zero(), Vector3::zero());
            }
        };
        
        struct SplineWithNormals{
            Vector3WithNormal endA, endB;
            
            SplineWithNormals(){}
            
            SplineWithNormals(Vector3WithNormal endA, Vector3WithNormal endB) : endA(endA), endB(endB){}
            
            Edge edge() const{
                return Edge(endA.x, endA.y, endB.x, endB.y);
            }
        };
        
        static float getHitRatio(const SplineWithNormals &intersected, float x){
            return ((x - intersected.endA.x) / (intersected.endB.x - intersected.endA.x));
        }
        
        static void getSplineDelta(const SplineWithNormals &intersected, float x, Vector3WithNormal &output){
            float hit = getHitRatio(intersected, x);
            output.x = x;
            output.z = intersected.endA.z + ((intersected.endB.z - intersected.endA.z) * hit);
            output.normal = (intersected.endA.normal + ((intersected.endB.normal - intersected.endA.normal) * hit)).normalized();
        }
        
        template <class Itr>
        static bool scanBounds(Itr begin, Itr end, float y, Vector3WithNormal &in, Vector3WithNormal &out){
            in.x = std::numeric_limits<float>::max();
            out.x = std::numeric_limits<float>::min();
            int found = 0;
            while (begin != end){
                //Edge edge(begin->edge());
                Vector2 direction(begin->edge().direction());
                float x = direction.y != 0.0f ? (((y * direction.x) - (begin->endA.y * direction.x))/ direction.y) + begin->endA.x : begin->endA.x;
                if (Edge::between(begin->endA.x, begin->endB.x, x)){
                    Vector3WithNormal v3;
                    getSplineDelta(*begin, x, v3);
                    if (x < in.x){
                        in = v3;
                    }
                    if (x > out.x){
                        out = v3;
                    }
                    ++found;
                }
                if (found == 2){
                    return true;
                }
                ++begin;
            }
            return found >= 1;
        }
        
        /*template <class Face>
        void fill(const Face &face, uint32_t colour){
            //auto list = face.vertices();
            auto edges = face.halfEdges();
            //BoundingRectangle rect(list.begin(), list.end());
            BoundingRectangle rect;
            rect.clear();
            for (auto i = face.halfEdges().begin(); i != face.halfEdges().end(); ++i){
                rect.add(i->vertex().position());
            }
            int h = height(), w = width();
            float stepSize = 1.0f / static_cast<float>(h);
            for (int y = (rect.topLeft.y * h), yend = (rect.bottomRight.y * h); y <= yend; ++y){
                if (y < 0 || y >= h){
                    continue;
                }
                float fy = static_cast<float>(y) * stepSize;
                float in, out;
                if (scanBounds(edges.begin(), edges.end(), fy, in, out)){
                    for (int x = std::max(static_cast<int>(in * w), 0), xend = std::min(static_cast<int>(out * w), w); x <= xend; ++x){
                        operator()(x, y) = colour;
                    }
                }
            }
        }*/
                
        void draw(const Edge &edge, uint32_t colour, int thickness = 1);
        
        /*template <class ColourFunc>
        void fillTriangle(const Vector3 &a, const Vector3 &b, const Vector3 &c, ColourFunc &colourFunc){
            Triangle triangle(Vector2(a.x, a.y), Vector2(b.x, b.y), Vector2(c.x, c.y));
            BoundingRectangle rect(triangle.boundingRectangle());
            int h = height(), w = width();
            float stepSize = 1.0f / static_cast<float>(h);
            for (int y = (rect.topLeft.y * h), yend = (rect.bottomRight.y * h); y <= yend; ++y){
                if (y < 0 || y >= h){
                    continue;
                }
                float fy = static_cast<float>(y) * stepSize;
                Spline splines[3];
                splines[0].endA = a;
                splines[0].endB = b;
                splines[1].endA = b;
                splines[1].endB = c;
                splines[2].endA = c;
                splines[2].endB = a;
                Vector3 in, out;
                if (scanBounds(splines, splines + 3, fy, in, out)){
                    int x = in.x * w, xend = out.x * w;
                    float zStep = (out.z - in.z) / (xend - x);
                    float xStep = (out.x - in.x) / (xend - x);
                    while (x < 0){
                        in.z += zStep;
                        in.x += xStep;
                        ++x;
                    }
                    xend = std::min(xend, w - 1);
                    while (x <= xend){
                        operator()(x, y) = colourFunc(in.x, fy, in.z);
                        in.z += zStep;
                        in.x += xStep;
                        ++x;
                    }
                }
            }
        }*/
        
        template <class ColourFunc>
        void fillTriangle(const Triangle3WithNormals &tri, ColourFunc &colourFunc){
            float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::min();
            Vector3WithNormal v3[3];
            for (int i = 0; i != 3; ++i){
                v3[i] = Vector3WithNormal(tri.vertices[i], tri.normals[i]);
                float y = tri.vertices[i].y;
                if (y < minY){
                    minY = y;
                }
                if (y > maxY){
                    maxY = y;
                }
            }
            int h = height(), w = width();
            float stepSize = 1.0f / static_cast<float>(h);
            for (int y = minY * h, yend = maxY * h; y <= yend; ++y){
                if (y < 0 || y >= h){
                    continue;
                }
                float fy = static_cast<float>(y) * stepSize;
                SplineWithNormals splines[3];
                splines[0].endA = v3[0];
                splines[0].endB = v3[1];
                splines[1].endA = v3[1];
                splines[1].endB = v3[2];
                splines[2].endA = v3[2];
                splines[2].endB = v3[0];
                Vector3WithNormal in, out;
                if (scanBounds(splines, splines + 3, fy, in, out)){
                    int x = in.x * w, xend = out.x * w;
                    float denom = xend - x;
                    Vector3WithNormal step(denom == 0.0f ? Vector3WithNormal::zero() : ((out - in) / denom));
                    while (x < 0){
                        in += step;
                        ++x;
                    }
                    xend = std::min(xend, w - 1);
                    while (x <= xend){
                        in.y = y * stepSize;
                        uint32_t &colour = operator()(x, y);
                        colour = colourFunc(x, y, colour, in);
                        in += step;
                        ++x;
                    }
                }
            }
        }
    };
}


#endif /* raster_h */
