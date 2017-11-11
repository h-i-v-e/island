//
//  terrain_graph.h
//  World Maker
//
//  Created by Jerome Johnson on 11/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef terrain_graph_h
#define terrain_graph_h

#include "voronoi_graph.h"
#include "grid.h"

namespace worldmaker{
    
    class TerrainGraph{
    public:
        TerrainGraph(size_t numVertices) : voronoi(numVertices) {}
        
        struct FaceData{
            bool sea;
        };
        
        struct VertexData;
        
        typedef VoronoiGraph<FaceData, VertexData> Voronoi;
        typedef typename Voronoi::HalfEdge HalfEdge;
        typedef typename Voronoi::Vertex Vertex;
        typedef typename Voronoi::Face Face;
        typedef typename Voronoi::HalfEdges HalfEdges;
        typedef typename Voronoi::Vertices Vertices;
        typedef typename Voronoi::Faces Faces;
        typedef Grid<Vector3> VertexGrid;
        
        struct VertexData{
            Vector3 normal;
            float z, seaDistance, landDistance;
            int flow;
            Vertex *down;
            
            VertexData() : flow(0), down(nullptr), z(0.0f){}
        };
        
        void generateTiles(unsigned int randomSeed, size_t numTiles, int relaxations);
        
        const Faces &faces() const{
            return voronoi.faces();
        }
        
        Faces &faces(){
            return voronoi.faces();
        }
        
        const Vertices &vertices() const{
            return voronoi.vertices();
        }
        
        Vertices &vertices() {
            return voronoi.vertices();
        }
        
        const HalfEdges &halfEdges() const{
            return voronoi.halfEdges();
        }
        
        HalfEdges &halfEdges() {
            return voronoi.halfEdges();
        }
        
        VertexGrid &copyTo(VertexGrid &) const;

    private:
        Voronoi voronoi;
    };
}

#endif /* terrain_graph_h */
