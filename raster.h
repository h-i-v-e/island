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
        
        template <class Itr>
        static bool scanBounds(Itr begin, Itr end, float y, float &in, float &out){
            in = std::numeric_limits<float>::max();
            out = std::numeric_limits<float>::min();
            int found = 0;
            while (begin != end){
                //Edge edge(begin->edge());
                Vector2 direction(begin->direction());
                float x = (((y * direction.x) - (begin->endA.y * direction.x))/ direction.y) + begin->endA.x;
                if (Edge::between(begin->endA.x, begin->endB.x, x)){
                    if (x < in){
                        in = x;
                    }
                    if (x > out){
                        out = x;
                    }
                    ++found;
                }
                ++begin;
            }
            return found >= 2;
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
        
        static float computeZ(const Vector2 &pos, const Vector3 &a, const Vector3 &b, const Vector3 &c){
            float aDist = (Vector2(a.x, a.y) - pos).magnitude();
            float bDist = (Vector2(b.x, b.y) - pos).magnitude();
            float cDist = (Vector2(c.x, c.y) - pos).magnitude();
            float total = aDist + bDist + cDist;
            aDist = total - aDist;
            bDist = total - bDist;
            cDist = total - cDist;
            return ((aDist * a.z) + (bDist * b.z) + (cDist * c.z)) / (aDist + bDist + cDist);
        }
        
        template <class ColourFunc>
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
                float in, out;
                Edge edges[3];
                triangle.getEdges(edges);
                if (scanBounds(edges, edges + 3, fy, in, out)){
                    float inZ = computeZ(Vector2(in, fy), a, b, c);
                    float outZ = computeZ(Vector2(out, fy), a, b, c);
                    int x = in * w, xend = out * w;
                    float zStep = (outZ - inZ) / (xend - x);
                    float xStep = (out - in) / (xend - x);
                    while (x < 0){
                        inZ += zStep;
                        in += xStep;
                        ++x;
                    }
                    xend = std::min(xend, w - 1);
                    while (x <= xend){
                        operator()(x, y) = colourFunc(in, fy, inZ);
                        inZ += zStep;
                        in += xStep;
                        ++x;
                    }
                }
            }

        }
    };
}


#endif /* raster_h */
