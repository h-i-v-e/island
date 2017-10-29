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
    
    class Raster;

    class Continent{
    public:
        struct FaceData{
            int distanceToSea;
        };
        
        struct VertexData{
            float z;
            int flow;
            void *down;
            
            VertexData() : flow(0){}
        };
        
        typedef VoronoiGraph<FaceData, VertexData> Graph;
        
        Continent(int numTiles) : numTiles(numTiles), graph(numTiles, numTiles){}
        
        void generateTiles(int relaxations);
        
        void generateSeasAndLakes(float waterRatio);
        
        void draw(Raster &raster) const;
        
    private:
        
        int numTiles, maxHeight;
        Graph graph;

};
}

#endif /* continent_h */
