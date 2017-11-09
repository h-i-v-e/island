//
//  erosian_map.cpp
//  World Maker
//
//  Created by Jerome Johnson on 9/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "erosian_map.h"
#include <random>
#include <iostream>

using namespace worldmaker;

namespace{
    int count = 0;
    
    void traceDrop(ErosianMap &em, int x, int y){
        float carrying = 0.0f, velocity = 0.0f;
        Vector3 *last = &em(x, y);
        if (last->z <= 0.0f){
            return;
        }
        for (;;){
            int xb = x - 1, xe = x + 1, yb = y - 1, ye = y + 1;
            if (xb < 0){
                xb = 0;
            }
            if (xe >= em.width()){
                xe = em.width();
            }
            if (yb < 0){
                yb = 0;
            }
            if (ye >= em.height()){
                ye = em.height();
            }
            float minZ = 0.0f;
            int nextX, nextY;
            while (yb <= ye){
                for (int i = xb; i <= xe; ++i){
                    if (i == x && yb == y){
                        continue;
                    }
                    Vector3 *cur = &em(i, yb);
                    if (cur->z >= last->z){
                        continue;
                    }
                    Vector3 dir(*cur - *last);
                    dir.normalize();
                    float z = dir.z;
                    if (z < minZ){
                        nextX = i;
                        nextY = yb;
                        minZ = z;
                    }
                }
                ++yb;
            }
            if (minZ < 0.0f){
                last = &em(nextX, nextY);
                float newVelocity = (velocity - minZ) * 0.5f;
                float canCarry = newVelocity * em.maxCarry();
                float add = carrying - canCarry;
                carrying = canCarry;
                velocity = newVelocity;
                if (nextX != x && nextY != y){
                    float lesser = add * 0.125f;
                    last->z += add * 0.75f;
                    em(nextX, y).z += lesser;
                    em(x, nextY).z += lesser;
                }
                else{
                    last->z += add;
                }
                //std::cout << "After " << last->z << std::endl;
                x = nextX;
                y = nextY;
            }
            else{
                //std::cout << "------- " << carrying << std::endl;
                /*if (++count == 100){
                    exit(0);
                }*/
                last->z += carrying;
                
                return;
            }
        }
    }
    
    void firstNormalPass(const ErosianMap &em, Grid<Vector3> &firstPass){
        int xEnd = em.width() - 1;
        int yEnd = em.height() - 1;
        Vector3 a(em(1, 1) - em(0, 0));
        Vector3 b(em(1, 0) - em(0, 1));
        firstPass(0, 0) = a.cross(b);
        for (int x = 1; x != xEnd; ++x){
            a = em(x + 1, 1) - em(x - 1, 0);
            b = em(x + 1, 0) - em(x - 1, 1);
            firstPass(x, 0) = a.cross(b);
        }
        a = em(xEnd, 1) - em(xEnd - 1, 0);
        b = em(xEnd, 0) - em(xEnd - 1, 1);
        firstPass(0, 0) = a.cross(b);
        for (int y = 1; y != yEnd; ++y){
            a = em(1, y + 1) - em(0, y - 1);
            b = em(1, y - 1) - em(0, y + 1);
            firstPass(0, y) = a.cross(b);
            for (int x = 1; x != xEnd; ++x){
                a = em(x + 1, y + 1) - em(x - 1, y - 1);
                b = em(x + 1, y - 1) - em(x - 1, y + 1);
                firstPass(x, y) = a.cross(b);
            }
            a = em(xEnd, y + 1) - em(xEnd - 1, y - 1);
            b = em(xEnd, y - 1) - em(xEnd - 1, y + 1);
            firstPass(0, y) = a.cross(b);
        }
        a = em(1, yEnd) - em(0, yEnd - 1);
        b = em(1, yEnd - 1) - em(0, yEnd);
        firstPass(0, yEnd) = a.cross(b);
        for (int x = 1; x != xEnd; ++x){
            a = em(x + 1, yEnd) - em(x - 1, yEnd - 1);
            b = em(x + 1, yEnd - 1) - em(x - 1, yEnd);
            firstPass(x, 0) = a.cross(b);
        }
        a = em(xEnd, yEnd) - em(xEnd - 1, yEnd - 1);
        b = em(xEnd, yEnd - 1) - em(xEnd - 1, yEnd);
        firstPass(xEnd, yEnd) = a.cross(b);
    }
}

void ErosianMap::erode(int iterations, int seed){
    std::uniform_int_distribution<int> wDist(0, mWidth - 1), hDist(0, mHeight - 1);
    std::mt19937 get(seed);
    while (iterations--){
        if (iterations % 10000 == 0){
            std::cout << iterations << std::endl;
        }
        traceDrop(*this, wDist(get), hDist(get));
    }
}

void ErosianMap::smooth(){
    Grid<float> zValues(mWidth, mHeight);
    for (int y = 0; y != mHeight; ++y){
        for (int x = 0; x != mWidth; ++x){
            zValues(x, y) = operator()(x, y).z;
        }
    }
    int xEnd = mWidth - 1, yEnd = mHeight - 1;
    float zMul = 1.0f / 9.0f;
    for (int y = 1; y != yEnd; ++y){
        for (int x = 1; x != xEnd; ++x){
            float total = 0.0f;
            for (int iy = y - 1, jy = y + 1; iy != jy; ++iy){
                for (int ix = x - 1, jx = x + 1; ix != jx; ++ix){
                    total += zValues(ix, iy);
                }
            }
            operator()(x, y).z = total * zMul;
        }
    }
}

void ErosianMap::calculateNormals(Grid<Vector3> &grid) const{
    Grid<Vector3> firstPass(mWidth, mHeight);
    firstNormalPass(*this, firstPass);
    int xEnd = mWidth - 1;
    int yEnd = mHeight - 1;
    for (int x = 0; x <= xEnd; ++x){
        grid(x, 0) = firstPass(x, 0).normalized();
    }
    for (int y = 1; y != yEnd; ++y){
        grid(0, y) = firstPass(0, y).normalized();
        for (int x = 1; x != xEnd; ++x){
            Vector3 normal(firstPass(x - 1, y - 1));
            normal += firstPass(x, y - 1);
            normal += firstPass(x + 1, y - 1);
            normal += firstPass(x - 1, y);
            normal += firstPass(x, y);
            normal += firstPass(x + 1, y);
            normal += firstPass(x - 1, y + 1);
            normal += firstPass(x, y + 1);
            normal += firstPass(x + 1, y + 1);
            grid(x, y) = normal.normalize();
        }
        grid(xEnd, y) = firstPass(xEnd, y).normalized();
    }
    for (int x = 0; x <= xEnd; ++x){
        grid(x, yEnd) = firstPass(x, yEnd).normalized();
    }
}
