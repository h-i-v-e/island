//
//  noise_layer.h
//  World Maker
//
//  Created by Jerome Johnson on 27/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef noise_layer_h
#define noise_layer_h

#include "simplex_noise.h"

namespace motu{
    class NoiseLayer{
    private:
        Vector2 offset;
        float scale;
        
    public:
        NoiseLayer(const Vector2 &offset, float scale) : offset(offset), scale(scale){}
        
        NoiseLayer(){}

		float getPlusMinus(const Vector2 &pos) const{
			Vector2 adjusted((pos + offset) * scale);
			return SimplexNoise::noise(adjusted.x, adjusted.y);
		}
        
        float get(const Vector2 &pos) const{
            return (1.0f + getPlusMinus(pos)) * 0.5f;
        }
    };
}

#endif /* noise_layer_h */
