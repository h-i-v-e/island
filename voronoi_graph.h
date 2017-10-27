//
//  voronoi_graph.h
//  World Maker
//
//  Created by Jerome Johnson on 21/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef voronoi_graph_h
#define voronoi_graph_h

#include "half_edge_pool.h"
#include "half_edge.h"

namespace worldmaker{
    
    class VoronoiGraph{
    private:
        HalfEdgePool<HalfEdge<>> pool;

    public:
        VoronoiGraph(size_t numVertices) : pool(numVertices) {}
        
        void generate(const HalfEdgePool<HalfEdge<>> &triangulation);
        
        const HalfEdgePool<HalfEdge<>> &halfEdges() const{
            return pool;
        }
    };
}

#endif /* voronoi_graph_h */
