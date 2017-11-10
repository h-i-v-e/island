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
        typedef typename Graph::HalfEdge HalfEdge;
        typedef typename Graph::Vertex Vertex;
        typedef typename Graph::Face Face;
        typedef std::vector<HalfEdge *> HalfEdges;
        typedef std::vector<HalfEdges> Coastlines;
        
        struct VertexData{
            Vector3 normal;
            float z, seaDistance, landDistance;
            int flow;
            Vertex *down;
            
            VertexData() : flow(0), down(nullptr), z(0.0f){}
        };
        
        Continent(int numTiles, float maxZ) : numTiles(numTiles), graph(numTiles, numTiles), maxZ(maxZ){}
        
        void generateTiles(int relaxations);
        
        void generateSeasAndLakes(float waterRatio);
        
        void generateRivers(int flowThreshold, int tesselations);
        
        void draw(Raster &raster);
        
        const Coastlines &coastlines() const{
            return mCoastlines;
        }
        
    private:
        
        void tesselate(HalfEdges &);
        
        void computeNormals();
        
        float maxZ;
        int numTiles, maxHeight;
        Graph graph;
        HalfEdges rivers;
        Coastlines mCoastlines;

};
}

#endif /* continent_h */
