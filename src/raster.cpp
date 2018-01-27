//
//  raster.cpp
//  World Maker
//
//  Created by Jerome Johnson on 25/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "edge.h"
#include "raster.h"
#include <algorithm>
#include "mesh.h"

using namespace motu;
using namespace std;

namespace{
    inline int GetOctant(int x1, int x2, int y1, int y2){
        if (x1 > x2) {
            if (y1 > y2) {
                return x1 - x2 > y1 - y2 ? 4 : 5;
            } else {
                return x1 - x2 > y2 - y1 ? 3 : 2;
            }
        } else if (y1 > y2) {
            return x2 - x1 > y1 - y2 ? 7 : 6;
        } else {
            return x2 - x1 > y2 - y1 ? 0 : 1;
        }
    }
    
    inline void SwitchToOctantZero(int octant, int &x, int &y){
        switch(octant){
            case 1:
                swap(x, y);
                break;
            case 2:
                swap(x, y);
                y = - y;
                break;
            case 3:
                x = -x;
                break;
            case 4:
                x = -x;
                y = -y;
                break;
            case 5:
                swap(x, y);
                x = -x;
                y = -y;
                break;
            case 6:
                swap(x, y);
                x = -x;
                break;
            case 7:
                y = -y;
                break;
        }
    }
    
    inline void SwitchFromOctantZero(int octant, int &x, int &y){
        switch(octant){
            case 1:
                swap(x, y);
                break;
            case 2:
                swap(x, y);
                x = - x;
                break;
            case 3:
                x = -x;
                break;
            case 4:
                x = -x;
                y = -y;
                break;
            case 5:
                swap(x, y);
                x = -x;
                y = -y;
                break;
            case 6:
                swap(x, y);
                y = -y;
                break;
            case 7:
                y = -y;
                break;
            default:
                break;
        }
    }
}

void Raster::draw(const Edge &edge, uint32_t colour){
    int w = width(), h = height();
    int ax = static_cast<int>(edge.endA.x * w),
    bx = static_cast<int>(edge.endB.x * w),
    ay = static_cast<int>(edge.endA.y * h),
    by = static_cast<int>(edge.endB.y * h);
    int octant = GetOctant (ax, bx, ay, by);
    SwitchToOctantZero (octant, ax, ay);
    SwitchToOctantZero (octant, bx, by);
    int dx = bx - ax, dy = by - ay;
    int twiceDx = dx << 1, twiceDy = dy << 1;
    int D = twiceDy - dx, y = ay;
    for (int x = ax; x <= bx; ++x) {
        int ox = x, oy = y;
        SwitchFromOctantZero (octant, ox, oy);
        if (ox >= 0 && ox < w && oy >= 0 && oy < h) {
            operator()(ox, oy) = colour;
        }
        if (D > 0) {
            ++y;
            D -= twiceDx;
        }
        D += twiceDy;
    }
}

void Raster::draw(const Mesh &mesh, uint32_t colour) {
	for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
		const Vector3 &a = mesh.vertices[mesh.triangles[i - 2]];
		const Vector3 &b = mesh.vertices[mesh.triangles[i - 1]];
		const Vector3 &c = mesh.vertices[mesh.triangles[i]];
		draw(Edge(a.x, a.y, b.x, b.y), colour);
		draw(Edge(b.x, b.y, c.x, c.y), colour);
		draw(Edge(c.x, c.y, a.x, a.y), colour);
	}
}

