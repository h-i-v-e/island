//
//  terrain_graph.h
//  World Maker
//
//  Created by Jerome Johnson on 11/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef terrain_graph_h
#define terrain_graph_h

//#include "voronoi_graph.h"
#include "dalauney_triangulation.h"
#include "grid.h"
#include "river.h"
#include <random>

namespace motu{
    
    class TerrainGraph{
    public:
        //TerrainGraph(size_t numVertices) : voronoi(numVertices) {}

		TerrainGraph(size_t numVertices) : mTriangulation(nullptr) {}

		~TerrainGraph() {
			delete mTriangulation;
		}
        
        struct FaceData{
            bool sea;
        };
        
        struct VertexData;
        
		typedef DalauneyTriangulation<FaceData, VertexData> Triangulation;
        //typedef VoronoiGraph<FaceData, VertexData> Voronoi;

        /*typedef typename Voronoi::HalfEdge HalfEdge;
        typedef typename Voronoi::Vertex Vertex;
        typedef typename Voronoi::Face Face;
        typedef typename Voronoi::HalfEdges HalfEdges;
        typedef typename Voronoi::Vertices Vertices;
        typedef typename Voronoi::Faces Faces;*/

		typedef typename Triangulation::HalfEdgeType HalfEdge;
		typedef typename Triangulation::Vertex Vertex;
		typedef typename Triangulation::Face Face;
		typedef typename Triangulation::HalfEdges HalfEdges;
		typedef typename Triangulation::Vertices Vertices;
		typedef typename Triangulation::Faces Faces;

        //typedef Grid<Vector3> VertexGrid;
        
        struct VertexData{
            Vector3 normal;
            float z, seaDistance, landDistance;
            int flow;
            Vertex *down;
            bool cliff;
            
            VertexData() : z(0.0f), flow(0), down(nullptr), cliff(false){}
        };
        
        void generateTiles(std::default_random_engine &, size_t numTiles, int relaxations);
        
        const Faces &faces() const{
            return mTriangulation->faces();
        }
        
        Faces &faces(){
            return mTriangulation->faces();
        }
        
        const Vertices &vertices() const{
            return mTriangulation->vertices();
        }
        
        Vertices &vertices() {
            return mTriangulation->vertices();
        }
        
        const HalfEdges &halfEdges() const{
            return mTriangulation->halfEdges();
        }
        
        HalfEdges &halfEdges() {
            return mTriangulation->halfEdges();
        }
        
        const Face &externalFace() const{
            return mTriangulation->externalFace();
        }

		Face &externalFace() {
			return mTriangulation->externalFace();
		}

    private:
        //Voronoi voronoi;
		Triangulation *mTriangulation;
    };
}

#endif /* terrain_graph_h */
