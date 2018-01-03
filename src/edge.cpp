//
//  edge.cpp
//  World Maker
//
//  Created by Jerome Johnson on 28/09/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "edge.h"
#include <algorithm>

using namespace motu;

bool Edge::intersection (const Edge &b, Vector2 &result) const{
    Vector3 va = parametric(), vb = b.parametric();
    if (va.x != 0.0f) {
        va.y /= va.x;
        va.z /= va.x;
        if (vb.x != 0.0f) {
            vb.y -= va.y * vb.x;
            vb.z -= va.z * vb.x;
        }
        if (vb.y == 0.0f) {
            return false;
        }
        vb.z /= vb.y;
        va.z -= vb.z * va.y;
        result.x = -va.z;
        result.y = -vb.z;
    } else if (vb.x != 0.0f) {
        vb.y /= vb.x;
        vb.z /= vb.x;
        if (va.y == 0.0f) {
            return false;
        }
        va.z /= va.y;
        vb.z -= va.z * vb.y;
        result.x = -vb.z;
        result.y = -va.z;
    } else {
        return false;
    }
    return true;
}
