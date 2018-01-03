//
//  river.cpp
//  World Maker
//
//  Created by Jerome Johnson on 13/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "river.h"
#include "edge.h"
#include <map>
#include <vector>
#include "brtree.h"
#include "circle.h"
#include <list>
#include "terrain_graph.h"
#include "triangle3.h"

namespace motu {
	struct River {
		std::list<RiverVertex*> sources;
		RiverVertex *mouth;
		int length;

		River(RiverVertex *mouth) : mouth(mouth), length(0) {}
	};
}

using namespace motu;

namespace{
    float computeShift(const Edge &a, const Edge &b, const Vector2 &src, const Vector2 &dst){
        return a.direction().normalized().dot(b.perp().normalized()) * (dst - src).magnitude();
    }
    
    float zAt(const Rivers::VertexGrid &grid, const Vector2 &vec){
        int x = grid.width() * vec.x, y = grid.height() * vec.y;
        if (x < 0){
            x = 0;
        }
        else if (x > grid.width()){
            x = grid.width();
        }
        if (y < 0){
            y = 0;
        }
        else if (y > grid.height()){
            y = grid.height();
        }
        return grid(x, y).z;
    }
    
    void ensureDownness(const Rivers::VertexGrid &grid, const Rivers::Sources &sources, std::map<Vector2, float> &depths){
        for (auto i = sources.begin(); i != sources.end(); ++i){
            float lastZ = zAt(grid, **i);
            for (const RiverVertex *vert = *i; vert; vert = vert->next()){
                auto j = depths.find(*vert);
                if (j == depths.end()){
                    j = depths.insert(j, std::make_pair(*vert, zAt(grid, *vert)));
                }
                if (j->second > lastZ){
                    j->second = lastZ;
                }
                else{
                    lastZ = j->second;
                }
            }
        }
    }
    
    Vector3 vec3(const Vector2 &vec2, float z){
        return Vector3(vec2.x, vec2.y, z);
    }
    
    bool scanBounds(const Triangle3 &tri, float y, Vector3 &in, Vector3 &out){
        in.x = std::numeric_limits<float>::max();
        out.x = std::numeric_limits<float>::min();
        int found = 0;
        for (int i = 0; i != 3; ++i){
            const Vector3 &endA = tri.vertices[i];
            const Vector3 &endB = tri.vertices[(i + 1) % 3];
            Vector2 direction(Edge(endA.x, endA.y, endB.x, endB.y).direction());
            float x = direction.y != 0.0f ? (((y * direction.x) - (endA.y * direction.x))/ direction.y) + endA.x : endA.x;
            if (Edge::between(endA.x, endB.x, x)){
                float hitAt = (x - endA.x) / (endB.x - endA.x);
                float z = endA.z + ((endB.z - endA.z) * hitAt);
                Vector3 v3(x, y, z);
                if (x < in.x){
                    in = v3;
                }
                if (x > out.x){
                    out = v3;
                }
                ++found;
            }
            if (found == 2){
                return true;
            }
        }
        return false;
    }
    
    void fillTriangle(Rivers::VertexGrid &grid, const Triangle3 &tri){
        float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::min();
        for (int i = 0; i != 3; ++i){
            float y = tri.vertices[i].y;
            if (y < minY){
                minY = y;
            }
            if (y > maxY){
                maxY = y;
            }
        }
        int h = grid.height(), w = grid.width();
        float stepSize = 1.0f / static_cast<float>(h);
        for (int y = minY * h, yend = maxY * h; y <= yend; ++y){
            if (y < 0 || y >= h){
                continue;
            }
            float fy = static_cast<float>(y) * stepSize;
            Vector3 in, out;
            if (scanBounds(tri, fy, in, out)){
                int x = in.x * w, xend = out.x * w;
                float denom = xend - x;
                if (denom == 0.0f){
                    continue;
                }
                Vector3 step((out - in) / denom);
                while (x < 0){
                    in += step;
                    ++x;
                }
                xend = std::min(xend, w - 1);
                while (x <= xend){
                    grid(x, y) = in;
                    in += step;
                    ++x;
                }
            }
        }
    }
    
    void carveRiverBeds(const Rivers::Meshes &meshes, const std::vector<Mesh::Vertices> &beds, Rivers::VertexGrid &grid){
        for (int i = 0; i != beds.size(); ++i){
            const Mesh &mesh = meshes[i];
            const Mesh::Vertices &bed = beds[i];
            for (int j = 2; j < mesh.triangles.size(); j += 3){
                fillTriangle(grid, Triangle3(
                                             bed[mesh.triangles[j - 2]],
                                             bed[mesh.triangles[j - 1]],
                                             bed[mesh.triangles[j]]
                                             )
                             );
            }
        }
    }

}

Rivers::Meshes &Rivers::getMeshesAndCarveRiverBeds(VertexGrid &grid, Meshes &meshes) const{
    std::map<Vector2, float> depths;
    ensureDownness(grid, mSources, depths);
    std::set<const RiverVertex *> added;
    float width = 1.0f / (grid.width() << 1);
    std::vector<Mesh::Vertices> riverBeds;
    for (auto i = mSources.begin(); i != mSources.end(); ++i){
        meshes.emplace_back();
        riverBeds.emplace_back();
        Edge edge;
        const RiverVertex *vert = *i;
        while (vert->mNext){
            if (added.find(vert) != added.end()){
                break;
            }
            edge(*vert, *vert->mNext);
            float riverWidth = width * sqrtf(vert->mFlow);
            float z = depths.find(*vert)->second;
            float bedZ = z;
            if (z >= 0.0f){
                bedZ -= riverWidth;
                //z -= riverWidth * 0.25f;
            }
            Vector2 bank(edge.perp().normalized() * riverWidth);
            meshes.back().vertices.emplace_back(vec3(*vert - bank, z));
            meshes.back().vertices.emplace_back(vec3(*vert + bank, z));
            riverBeds.back().emplace_back(vec3(*vert - bank, bedZ));
            riverBeds.back().emplace_back(vec3(*vert + bank, bedZ));
            added.insert(vert);
            vert = vert->mNext;
        }
        float riverWidth = width * sqrtf(vert->mFlow);
        float z = depths.find(*vert)->second;
        float bedZ = z - riverWidth;
        //z -= riverWidth;
        Vector2 bank(edge.perp().normalized() * riverWidth);
        meshes.back().vertices.emplace_back(vec3(*vert - bank, z));
        meshes.back().vertices.emplace_back(vec3(*vert + bank, z));
        riverBeds.back().emplace_back(vec3(*vert - bank, bedZ));
        riverBeds.back().emplace_back(vec3(*vert + bank, bedZ));
        added.insert(vert);
        for (int i = 2; i < meshes.back().vertices.size(); ++i){
            if (i & 1){
                meshes.back().triangles.push_back(i - 1);
                meshes.back().triangles.push_back(i - 2);
                meshes.back().triangles.push_back(i);
            }
            else{
                meshes.back().triangles.push_back(i - 2);
                meshes.back().triangles.push_back(i - 1);
                meshes.back().triangles.push_back(i);
            }
        }
    }
    carveRiverBeds(meshes, riverBeds, grid);
    return meshes;
}

Rivers::Meshes &Rivers::correctMeshHeight(VertexGrid &grid, Meshes &meshes) const{
    /*std::map<Vector2, float> depths;
    ensureDownness(grid, mSources, depths);*/
    for (auto i = meshes.begin(); i != meshes.end(); ++i){
        for (int j = 1; j < i->vertices.size(); j += 2){
            Vector3 &a = i->vertices[j - 1], &b = i->vertices[j];
            float adjust = (b - a).magnitude();
            a.z = zAt(grid, Vector2(a.x, a.y)) + adjust;
            b.z = zAt(grid, Vector2(b.x, b.y)) + adjust;
        }
        /*for (auto j = i->vertices.begin(); j != i->vertices.end(); ++j){
            float width =
            j->z = depths.find(Vector2(j->x, j->y))->second;
        }*/
    }
    return meshes;
}

void Rivers::tesselate(RiverVertex *source, VertexSet &added){
    if (!source->mNext || !source->mNext->mNext){
        return;
    }
    Edge last(*source, *source->mNext);
    Edge current(*source->mNext, *source->mNext->mNext);
    source->insert(vertices.allocate(), computeShift(last, current, *source, *source->mNext));
    source = source->mNext;
    while (source->mNext->mNext->mNext){
        if (added.find(source->mNext) != added.end()){
            return;
        }
        added.insert(source->mNext);
        last = Edge(*source, *source->mNext);
        current = Edge(*source->mNext->mNext, *source->mNext->mNext->mNext);
        source->mNext->insert(vertices.allocate(), computeShift(last, current, *source->mNext, *source->mNext->mNext));
        source = source->mNext->mNext;
    }
    if (added.find(source->mNext) != added.end()){
        return;
    }
    added.insert(source->mNext);
    last = Edge(*source, *source->mNext);
    current = Edge(*source->mNext, *source->mNext->mNext);
    source->mNext->insert(vertices.allocate(), computeShift(last, current, *source->mNext, *source->mNext->mNext));
}

void Rivers::construct(const Edges &edges, int tesselations, float minSeperation){
    std::map<Vector2, RiverVertex *> vertexMap;
    for (const Edge &edge : edges){
        RiverVertex *add;
        auto i = vertexMap.find(edge.endA);
        if (i == vertexMap.end()){
            add = vertices.allocate(edge.endA.x, edge.endA.y);
            add->mFlow = 1.0f;
            vertexMap.emplace(edge.endA, add);
        }
        else{
            add = i->second;
        }
        i = vertexMap.find(edge.endB);
        if (i != vertexMap.end()){
            add->mNext = i->second;
            for (RiverVertex *v = i->second; v; v = v->mNext){
                v->mFlow += add->mFlow;
            }
        }
        else{
            add->mNext = vertices.allocate(edge.endB.x, edge.endB.y);
            add->mNext->mFlow = add->mFlow + 1;
            vertexMap.emplace(edge.endB, add->mNext);
        }
    }
    for (auto i = vertexMap.begin(); i != vertexMap.end(); ++i){
        if (i->second->mFlow == 1){
            mSources.push_back(i->second);
        }
    }
    merge(minSeperation);
    VertexSet added;
    for (auto i = mSources.begin(); i != mSources.end(); ++i){
        for (int j = 0; j != tesselations; ++j){
            added.clear();
            tesselate(*i, added);
        }
    }
}

void Rivers::merge(float minSeperation){
    std::multimap<RiverVertex *, std::pair<RiverVertex *, int>> riverEnds;
    int totalVertices = 0;
    for (auto i = mSources.begin(); i != mSources.end(); ++i){
        RiverVertex *mouth = *i;
        int count = 0;
        for (RiverVertex *down = mouth; down; down = down->next()){
            mouth = down;
            ++count;
        }
        totalVertices += count;
        riverEnds.emplace(mouth, std::make_pair(*i, count));
    }
    //std::vector<River> rivers;
    typedef BRTree<RiverVertex*> Tree;
    Tree tree(totalVertices);
    float halfSep = minSeperation * 0.5f;
    for (auto i = mSources.begin(); i != mSources.end(); ++i){
        for (RiverVertex *down = *i; down; down = down->next()){
            tree.add(Circle(*down, halfSep).boundingRectangle(), down);
        }
    }
    std::vector<River> mouths;
    for (auto i = riverEnds.begin(); i != riverEnds.end();){
        auto j = i;
        mouths.emplace_back(i->first);
        while (i != riverEnds.end() && i->first == j->first){
            mouths.back().sources.emplace_back(i->second.first);
            mouths.back().length += i->second.second;
            ++i;
        }
    }
    std::sort(mouths.begin(), mouths.end(), [](const River &a, const River &b){
        return a.length < b.length;
    });
    Tree::Values values;
    float sqrSep = minSeperation * minSeperation;
    for (auto i = mouths.begin(); i != mouths.end(); ++i){
        for (auto j = i->sources.begin(); j != i->sources.end(); ++j){
            for (RiverVertex *vert = *j; vert; vert = vert->mNext){
                tree.remove(Circle(*vert, halfSep).boundingRectangle(), vert);
            }
        }
        for (auto j = i->sources.begin(); j != i->sources.end(); ++j){
            RiverVertex *last = *j;
            for (RiverVertex *vert = last; vert; vert = vert->mNext){
                Circle comp(*vert, halfSep);
                 for (auto k = tree.intersecting(comp.boundingRectangle(), values); k.first != k.second; ++k.first){
                     if (*k.first != vert && (**k.first - *vert).sqrMagnitude() <= sqrSep){
                         last->mNext = *k.first;
                         vert->mNext = nullptr;
                         break;
                     }
                 }
                last = vert;
            }
        }
    }
    for (auto i = mouths.begin(); i != mouths.end(); ++i){
        for (RiverVertex *vert = i->sources.front(); vert->mNext; vert = vert->mNext){
            if (vert->mNext == i->mouth){
                *i->mouth += (*i->mouth - *vert);
                break;
            }
        }
    }
}

