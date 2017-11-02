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
            bool sea;
        };
        
        struct VertexData;
        
        typedef VoronoiGraph<FaceData, VertexData> Graph;
        typedef typename Graph::HalfEdges::ObjectType HalfEdge;
        typedef typename Graph::Vertices::ObjectType Vertex;
        typedef typename Graph::Faces::ObjectType Face;
        
        struct VertexData{
            float z;
            int flow;
            Vertex *down;
            
            VertexData() : flow(0){}
        };
        
        typedef std::vector<HalfEdge *> HalfEdges;
        
        Continent(int numTiles) : numTiles(numTiles), graph(numTiles, numTiles){}
        
        void generateTiles(int relaxations);
        
        void generateSeasAndLakes(float waterRatio);
        
        void generateRivers(int flowThreshold, int tesselations);
        
        void draw(Raster &raster) const;
        
    private:
        
        void tesselate(HalfEdges &);
        
        int numTiles, maxHeight;
        Graph graph;
        HalfEdges rivers;

};
}

#endif /* continent_h */
