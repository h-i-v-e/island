//
//  continent.h
//  World Maker
//
//  Created by Jerome Johnson on 25/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef continent_h
#define continent_h

#include "half_edge.h"
#include "voronoi_graph.h"

namespace worldmaker{
    
    class VoronoiGraph;
    class Raster;

    class Continent{
    private:
    
        typedef std::map<HalfEdge<>::Face, bool> Tiles;
    
        int numTiles;
        VoronoiGraph graph;
        Tiles tiles;
    
    public:
        Continent(int numTiles) : numTiles(numTiles), graph(numTiles){}
        
        void generateTiles(int relaxations);
        
        void generateSeasAndLakes(float waterRatio);
        
        void draw(Raster &raster) const;
};
}

#endif /* continent_h */
