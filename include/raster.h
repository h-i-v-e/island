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
namespace motu{

	struct Mesh;
	struct Edge;
    
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
                
        void draw(const Edge &edge, uint32_t colour);

		void draw(const Mesh &mesh, uint32_t colour);
    };
}


#endif /* raster_h */
