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
#include "terrain_graph.h"

namespace motu{
    
    class Raster;

    class Continent{
    public:
        
        Continent(std::default_random_engine &rnd, int numTiles, int relaxations, float maxZ) : graph(numTiles), maxZ(maxZ){
            graph.generateTiles(rnd, numTiles, relaxations);
        }
        
        //void generateTiles(int relaxations);
        
        void generateSeasAndLakes(std::default_random_engine &, float waterRatio);
        
        //void generateRivers(int flowThreshold, int tesselations);
        
        void draw(Raster &raster);
        
        /*const Coastlines &coastlines() const{
            return mCoastlines;
        }*/
        
    private:
        
        //void tesselate(HalfEdges &);
        
        void computeNormals();
        
        float maxZ, maxHeight;
        TerrainGraph graph;
        //HalfEdges rivers;
        //Coastlines mCoastlines;

};
}

#endif /* continent_h */
