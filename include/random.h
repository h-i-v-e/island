//
//  random.h
//  World Maker
//
//  Created by Jerome Johnson on 7/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef random_h
#define random_h

#include <random>
#include "vector2.h"

namespace motu {
    struct Random{
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_real_distribution<float> dis;
        
        Random() : gen(rd()), dis(0.0f, 1.0f){
        }
        
        Vector2 vector2(){
            return Vector2(dis(gen), dis(gen));
        }
        
        uint32_t colour(){
            return (static_cast<uint32_t>(0xffffff * dis(gen)) << 8) | 0xff;
        }
        
        float get(){
            return dis(gen);
        }
    };
}

#endif /* random_h */
