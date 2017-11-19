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
#include <memory>
#include <set>

using namespace worldmaker;

namespace{
    
    typedef std::set<uint64_t> VisitedCliffs;
    
    constexpr uint64_t makeKey(uint32_t x, uint32_t y){
        return (static_cast<uint64_t>(y) << 32) | x;
    }
    
    inline void getKey(uint64_t value, int &x, int &y){
        x = value & 0xffffffff;
        y = value >> 32;
    }
    
    int isCliff(/*VisitedCliffs &visited, */ErosianMap &map, uint32_t x, uint32_t y){
        if (map(x, y).z <= 0.0f){
            return -2;
        }
        uint8_t landCount = 0, seaCount = 0;
        for (uint32_t i = y - 1; i <= y + 1; ++i){
            for (uint32_t j = x - 1; j <= x + 1; ++j){
                if (i == y && j == x){
                    continue;
                }
                if (map(j, i).z < 0.0f){
                    ++seaCount;
                }
                else{
                    ++landCount;
                }
            }
        }
        if (seaCount == 0 || landCount == 0){
            return -2;
        }
        if (seaCount > landCount){
            return 1;
        }
        //else if (seaCount < landCount){
            return -1;
        //}
        /*landCount = seaCount = 0;
        for (int i = y - 1; i <= y + 1; ++i){
            for (int j = x - 1; j <= x + 1; ++j){
                uint64_t key = makeKey(j, i);
                if (visited.find(key) != visited.end()){
                    continue;
                }
                else{
                    visited.insert(key);
                }
                switch(isCliff(visited, map, j, i)){
                    case -1:
                        ++landCount;
                    case 1:
                        ++seaCount;
                }
            }
        }
        return (seaCount > landCount) ? -1 : 1;*/
    }
    
    void blendCliff(ErosianMap &map, VisitedCliffs &visited, uint32_t x, uint32_t y, float value, float minValue){
        static float diagonalWeight = 1.0f / sqrtf(2.0f);
        static float halfDiagonalWeight = diagonalWeight * 0.5f;
        //float &z = map(x, y).z;
        //z = (z < 0.0f) ? -value : value;
        map(x, y).z += value;
        value *= 0.1f;
        if (value < minValue){
            return;
        }
        for (uint32_t i = y - 1; i <= y + 1; ++i){
            for (uint32_t j = x - 1; j <= x + 1; ++j){
                if ((i == y && j == x) || (map(j, i).z < 0.0f)){
                    continue;
                }
                uint64_t key = makeKey(j, i);
                if (visited.find(key) != visited.end()){
                    continue;
                }
                /*visited.insert(key);*/
                if (j != x && i != y){
                    float lesser = /*(map(j, i).z <= 0.0f ? -value : value)*/value * halfDiagonalWeight;
                    map(j, y).z += lesser;
                    map(x, i).z += lesser;
                    value *= diagonalWeight;
                }
                blendCliff(map, visited, j, i, value, minValue);
            }
        }
    }
    
    void traceDrop(ErosianMap &em, int x, int y){
        static float diagonalWeight = 1.0f / sqrtf(2.0f);
        static float halfDiagonalWeight = diagonalWeight * 0.5f;
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
                    float z = dir.z;
                    if (i != x && yb != y){
                        z *= diagonalWeight;
                    }
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
                    float lesser = add * halfDiagonalWeight;
                    last->z += add * diagonalWeight;
                    em(nextX, y).z += lesser;
                    em(x, nextY).z += lesser;
                }
                else{
                    last->z += add;
                }
                x = nextX;
                y = nextY;
            }
            else{
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
    
    struct DownTracker{
        int x, y, flow;
        
        DownTracker() : x(-1), y(-1), flow(0){}
    };
    
    typedef Grid<DownTracker> FlowGrid;
}

void ErosianMap::erode(int iterations, int seed){
    std::uniform_int_distribution<int> wDist(0, mWidth - 1), hDist(0, mHeight - 1);
    std::mt19937 get(seed);
    while (iterations--){
#ifdef DEBUG
        if (iterations % 10000 == 0){
            std::cout << iterations << std::endl;
        }
#endif
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
    //float zMul = 1.0f / 9.0f;
    for (int y = 1; y != yEnd; ++y){
        for (int x = 1; x != xEnd; ++x){
            float total = 0.0f;
            int count = 0;
            bool sea = zValues(x, y) < 0.0f;
            for (int iy = y - 1, jy = y + 1; iy != jy; ++iy){
                for (int ix = x - 1, jx = x + 1; ix != jx; ++ix){
                    float z = zValues(ix, iy);
                    if ((sea && z < 0.0f) || ((!sea) && z >= 0.0f)){
                        total += z;
                        ++count;
                    }
                }
            }
            operator()(x, y).z = total / count;
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

void ErosianMap::trackRivers(Rivers &rivers, int threshold) const{
    static float diagonalWeight = 1.0f / sqrtf(2.0f);
    FlowGrid down(mWidth, mHeight);
    int yEnd = mHeight - 1, xEnd = mWidth - 1;
    for (int y = 1; y != yEnd; ++y){
        for (int x = 1; x != xEnd; ++x){
            DownTracker *chosen = &down(x, y);
            const Vector3 *cur = &operator()(x, y);
            if (cur->z < 0.0f || cur->x == 0.0f || cur->y == 0.0f){
                continue;
            }
            float lowest = 0.0f;
            for (int iy = y - 1, jy = y + 1; iy <= jy; ++iy){
                for (int ix = x - 1, jx = x + 1; ix <= jx; ++ix){
                    if (iy == y && ix == x){
                        continue;
                    }
                    const Vector3 *next = &operator()(ix, iy);
                    if (cur->z <= next->z){
                        continue;
                    }
                    float z = (*next - *cur).z;
                    if (ix != x && iy != y){
                        z *= diagonalWeight;
                    }
                    if (z < lowest){
                        lowest = z;
                        chosen->x = ix;
                        chosen->y = iy;
                    }
                }
            }
        }
    }
    for (int y = 1; y != yEnd; ++y){
        for (int x = 1; x != xEnd; ++x){
            for (DownTracker *chosen = &down(x, y); chosen->x != -1; chosen = &down(chosen->x, chosen->y)){
                ++chosen->flow;
            }
        }
    }
    for (const DownTracker *i = down.data(), *j = down.data() + (mWidth * mHeight); i != j; ++i){
        if (i->flow >= threshold){
            rivers.emplace_back(i->x, i->y);
        }
    }
}


void ErosianMap::raiseCliffs(int steps){
    //amount *= 0.5f;
    //auto visited = std::unique_ptr<Grid<bool>>(new Grid<bool>(mWidth, mHeight));
    //visited->zero();
    int lastX = mWidth - 1, lastY = mHeight - 1;
    //float minAmount = amount * 0.001f;
    VisitedCliffs visitedCliffs;
    for (int i = 0; i != steps; ++i){
        visitedCliffs.clear();
        for (int y = 1; y != lastY; ++y){
            for (int x = 1; x != lastX; ++x){
                if (isCliff(*this, x, y) == 1){
                    visitedCliffs.insert(makeKey(x, y));
                }
            }
        }
        for (auto i : visitedCliffs){
            int x, y;
            getKey(i, x, y);
            operator()(x, y).z = -0.001f;
        /*blendCliff(*this, visitedCliffs, x, y, amount, minAmount);*/
        }
    }
}
