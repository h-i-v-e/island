//
//  dalauney_triangulation.h
//  World Maker
//
//  Created by Jerome Johnson on 14/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef dalauney_triangulation_h
#define dalauney_triangulation_h

#include "vector2.h"
#include "half_edge_pool.h"
#include "half_edge.h"

namespace worldmaker{
    
    class DalauneyTriangulation{
    private:
        HalfEdgePool<HalfEdge<>> *pool;
        
    public:
        
        DalauneyTriangulation(const std::vector<Vector2> &points);
        
        ~DalauneyTriangulation();
        
        const HalfEdgePool<HalfEdge<>> &halfEdges() const{
            return *pool;
        }

    };
}

#endif /* dalauney_triangulation_h */
