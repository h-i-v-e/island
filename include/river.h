//
//  river.h
//  World Maker
//
//  Created by Jerome Johnson on 13/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef river_h
#define river_h

#include <vector>
#include "object_pool.h"
#include "edge.h"
#include <set>
#include "mesh.h"
#include "grid.h"

namespace motu{
    
    class RiverVertex : public Vector2{
    public:
        friend struct River;
        friend class Rivers;
        
        constexpr const RiverVertex *next() const{
            return mNext;
        }
        
        RiverVertex *next() {
            return mNext;
        }
        
        constexpr float flow() const{
            return mFlow;
        }
        
        RiverVertex(float x, float y) : Vector2(x, y), mFlow(0.0f), mNext(nullptr){}
        
        RiverVertex() : mFlow(0.0f), mNext(nullptr){}
        
    private:
        
        void insert(RiverVertex *insert, float shift){
            insert->mFlow = (mFlow + mNext->mFlow) * 0.5f;
            Edge edge(*this, *mNext);
            insert->mNext = mNext;
            mNext = insert;
            Vector2 pos(edge.midPoint() + (edge.normal() * shift * 10.0f));
            insert->x = pos.x;
            insert->y = pos.y;
        }
        
        float mFlow;
        RiverVertex *mNext;
    };
    
    class Rivers{
    public:
        typedef std::vector<Edge> Edges;
        typedef std::vector<RiverVertex*> Sources;
        
        Rivers(const Edges &edges, int tesselations, float minSeperation) : vertices(edges.size() * (tesselations + 1)){
            construct(edges, tesselations, minSeperation);
        }
        
        const Sources &sources() const{
            return mSources;
        }
        
        typedef std::vector<Mesh> Meshes;
        typedef Grid<Vector3> VertexGrid;
        
        Meshes &getMeshesAndCarveRiverBeds(VertexGrid &, Meshes &meshes) const;
        
        Meshes &correctMeshHeight(VertexGrid &, Meshes &meshes) const;
        
    private:
        
        void construct(const Edges &edges, int tesselations, float minSeperation);
        
        typedef std::set<RiverVertex *> VertexSet;
        
        void tesselate(RiverVertex *source, VertexSet &added);
        
        void merge(float minSeperation);
        
        typedef ObjectPool<RiverVertex> Vertices;
        
        Vertices vertices;
        Sources mSources;
    };
}

#endif /* river_h */
