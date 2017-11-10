//
//  spline.h
//  World Maker
//
//  Created by Jerome Johnson on 3/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef spline_h
#define spline_h

#include "vector3.h"
#include "edge.h"

namespace worldmaker{
    struct Spline{
        Vector3 endA, endB;
        
        Spline(){}
        
        Spline(const Vector3 &endA, const Vector3 &endB) : endA(endA), endB(endB){}
        
        Edge edge() const{
            return Edge(endA.x, endA.y, endB.x, endB.y);
        }
    };
}

#endif /* spline_h */