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
                Edge edge(begin->edge());
                Vector2 direction(edge.direction());
                float x = (((y * direction.x) - (edge.endA.y * direction.x))/ direction.y) + edge.endA.x;
                if (Edge::between(edge.endA.x, edge.endB.x, x)){
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
        
        template <class Face>
        void fill(const Face &face, uint32_t colour){
            auto list = face.vertices();
            auto edges = face.halfEdges();
            BoundingRectangle rect(list.begin(), list.end());
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
        }
                
        void draw(const Edge &edge, uint32_t colour, int thickness = 1);
    };
}


#endif /* raster_h */
