//
//  grid.h
//  World Maker
//
//  Created by Jerome Johnson on 9/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef grid_h
#define grid_h

namespace motu{
    template <class ValueType>
    class Grid{
    protected:
        int mWidth, mHeight;
        ValueType *mData;
    public:
        Grid(int width, int height) : mWidth(width), mHeight(height), mData(new ValueType[width * height]){}

		ValueType *detach() {
			ValueType *out = mData;
			mData = nullptr;
			return out;
		}
        
        ~Grid(){
            delete [] mData;
        }
        
        void zero(){
            memset(mData, 0, mWidth * mHeight * sizeof(ValueType));
        }
        
        constexpr int width() const{
            return mWidth;
        }
        
        constexpr int height() const{
            return mHeight;
        }
        
        ValueType &operator()(int x, int y){
            return mData[(y * mWidth) + x];
        }
        
        constexpr ValueType &operator()(int x, int y) const{
            return mData[(y * mWidth) + x];
        }
        
        const ValueType *data() const{
            return mData;
        }

		ValueType *data() {
			return mData;
		}

		constexpr int length() const {
			return mWidth * mHeight;
		}
    };
}

#endif /* grid_h */
