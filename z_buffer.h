//
//  z_buffer.h
//  World Maker
//
//  Created by Jerome Johnson on 9/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef z_buffer_h
#define z_buffer_h

namespace worldmaker{
    class ZBuffer{
    public:
        ZBuffer(int width, int height) : mWidth(width), mHeight(height), mData(new float[width * height]){}
        
        ~ZBuffer(){
            delete [] mData;
        }
        
        float &operator()(int x, int y){
            return mData[(y * mWidth) + x];
        }
        
        constexpr float operator()(int x, int y) const{
            return mData[(y * mWidth) + x];
        }
        
        float &operator[](int i){
            return mData[i];
        }
        
        constexpr float operator[](int i) const{
            return mData[i];
        }
        
        constexpr int width() const{
            return mWidth;
        }
        
        constexpr int height() const{
            return mHeight;
        }
        
        constexpr int length() const{
            return mWidth * mHeight;
        }
    private:
        int mWidth, mHeight;
        float *mData;
    };
}

#endif /* z_buffer_h */
