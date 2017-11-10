//
//  erosian_map.h
//  World Maker
//
//  Created by Jerome Johnson on 9/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef erosian_map_h
#define erosian_map_h

#include "grid.h"
#include "vector3.h"
#include <vector>

namespace worldmaker{
    class ErosianMap : public Grid<Vector3>{
    public:
        ErosianMap(int width, int height, float maxCarry) : Grid<Vector3>(width, height), mMaxCarry(maxCarry){}
        
        void erode(int iterations, int seed);
        
        void smooth();
        
        void calculateNormals(Grid<Vector3> &grid) const;
        
        typedef std::vector<std::pair<int, int>> Rivers;
        
        void trackRivers(Rivers &rivers, int threshold) const;
        
        constexpr float maxCarry() const{
            return mMaxCarry;
        }
    private:
        float mMaxCarry;
    };
}

#endif /* erosian_map_h */
