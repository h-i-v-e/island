//
//  half_edge.h
//  World Maker
//
//  Created by Jerome Johnson on 16/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef half_edge_h
#define half_edge_h

#include <algorithm>
#include "edge.h"
#include <set>
#include <map>

namespace worldmaker{
    
    struct EmptyData{
    };
    
    template<class FaceData = EmptyData, class VertexData = EmptyData>
    class HalfEdge
    {
    public:
        typedef std::set<const HalfEdge*> Set;
        
        class HalfEdgeIteratorBase{
        protected:
            const HalfEdge *first;
            bool incremented;
            
            HalfEdgeIteratorBase(const HalfEdge *first, bool incremented) : first(first), incremented(incremented){}
            
        public:
            
            bool operator==(const HalfEdgeIteratorBase &other) const{
                return first == other.first && incremented == other.incremented;
            }
            
            bool operator!=(const HalfEdgeIteratorBase &other) const{
                return first && other.first && (first != other.first || incremented != other.incremented);
            }
            
            const HalfEdge *current() const{
                return first;
            }
        };
        
        class VertexIteratorBase : public HalfEdgeIteratorBase{
            protected:
                VertexIteratorBase(const HalfEdge *first, bool incremented) : HalfEdgeIteratorBase(first, incremented){}
            public:
                const Vector2 &operator*(){
                    return HalfEdgeIteratorBase::first->vector2;
                }
            
                const Vector2 *operator->(){
                    return &HalfEdgeIteratorBase::first->vector2;
                }
        };
        
        template<class Type, class IteratorType>
        class Iterable{
        protected:
            const Type *type;
            
            Iterable(const Type *type) : type(type){}
        public:
            IteratorType begin() const{
                return IteratorType(type, false);
            }
            
            IteratorType end() const{
                return IteratorType(type, true);
            }
        };

        class Face{
        public:
            friend class HalfEdge;
            
            struct HalfEdgeIterator : public HalfEdgeIteratorBase{
                HalfEdgeIterator(const HalfEdge *edge, bool incremented) : HalfEdgeIteratorBase(edge, incremented){}
                
                HalfEdgeIterator &operator++(){
                    HalfEdgeIteratorBase::first = HalfEdgeIteratorBase::first->next;
                    HalfEdgeIteratorBase::incremented = true;
                    return *this;
                }
                
                HalfEdgeIterator operator++(int){
                    HalfEdgeIterator out(HalfEdgeIteratorBase::first, HalfEdgeIteratorBase::incremented);
                    HalfEdgeIteratorBase::first = HalfEdgeIteratorBase::first->next;
                    HalfEdgeIteratorBase::incremented = true;
                    return out;
                }
                
                const HalfEdge &operator*() const{
                    return *HalfEdgeIteratorBase::first;
                }
                
                const HalfEdge *operator->() const{
                    return HalfEdgeIteratorBase::first;
                }
            };
            
            struct VertexIterator : public VertexIteratorBase{
                VertexIterator(const HalfEdge *edge, bool incremented) : VertexIteratorBase(edge, incremented){}
                
                VertexIterator &operator++(){
                    HalfEdgeIteratorBase::first = HalfEdgeIteratorBase::first->next;
                    HalfEdgeIteratorBase::incremented = true;
                    return *this;
                }
                
                VertexIterator operator++(int){
                    VertexIterator out(HalfEdgeIteratorBase::first, HalfEdgeIteratorBase::incremented);
                    HalfEdgeIteratorBase::first = HalfEdgeIteratorBase::first->next;
                    HalfEdgeIteratorBase::incremented = true;
                    return out;
                }
            };
            
            class HalfEdgeIterable : public Iterable<HalfEdge, HalfEdgeIterator>{
            private:
                HalfEdgeIterable(const HalfEdge *edge) : Iterable<HalfEdge, HalfEdgeIterator>(edge){}
            public:
                friend class Face;
            };
            
            class VertexIterable : public Iterable<HalfEdge, VertexIterator>{
            private:
                VertexIterable(const HalfEdge *edge) : Iterable<HalfEdge, VertexIterator>(edge){}
            public:
                friend class Face;
            };
            
        private:
            
            HalfEdge *edge;
            FaceData mData;
            
        public:
            typedef std::set<Face> Set;
            
            Face (HalfEdge *edge) : edge(edge){
            }
            
            Face() : edge(nullptr){}
            
            HalfEdgeIterable halfEdges() const{
                return HalfEdgeIterable(edge);
            }
            
            VertexIterable vertices() const{
                return VertexIterable(edge);
            }
            
            FaceData &data(){
                return mData;
            }
            
            const FaceData &data() const{
                return mData;
            }
            
            const HalfEdge *halfEdge() const{
                return edge;
            }
            
            HalfEdge *halfEdge(){
                return edge;
            }
            
            bool operator < (const Face &face) const{
                return edge < face.edge;
            }
            
            Vector2 calculateCentroid() const;
        };
        
        class Vertex{
        private:
            const HalfEdge *edge;
            VertexData mData;
            
        public:
            
            Vertex (const HalfEdge *edge) : edge(edge){}
            
            Vertex () : edge(nullptr){}
            
            friend class HalfEdge;
            
            const Vector2 &position() const{
                return edge->vector2;
            }
            
            bool complete() const{
                const HalfEdge *e = edge;
                do {
                    if (!e->pair){
                        return false;
                    }
                    e = e->pair->next;
                } while (e != edge);
                return true;
            }
            
            VertexData &data(){
                return mData;
            }
            
            const VertexData &data() const{
                return mData;
            }
            
            struct InboundHalfEdgeIterator : public HalfEdgeIteratorBase{
                InboundHalfEdgeIterator(const HalfEdge *edge, bool incremented) : HalfEdgeIteratorBase(edge, incremented){}
                
                InboundHalfEdgeIterator &operator++(){
                    if (HalfEdgeIteratorBase::first->pair){
                        HalfEdgeIteratorBase::first = HalfEdgeIteratorBase::first->pair->next;
                    }
                    else{
                        HalfEdgeIteratorBase::first = nullptr;
                    }
                    HalfEdgeIteratorBase::incremented = true;
                    return *this;
                }
                
                InboundHalfEdgeIterator operator++(int){
                    InboundHalfEdgeIterator out(HalfEdgeIteratorBase::first, HalfEdgeIteratorBase::incremented);
                    ++*this;
                    return out;
                }
                
                const HalfEdge &operator*() const{
                    return *HalfEdgeIteratorBase::first;
                }
                
                const HalfEdge *operator->() const{
                    return HalfEdgeIteratorBase::first;
                }
            };
            
            class InboundHalfEdgeIterable : public Iterable<HalfEdge, InboundHalfEdgeIterator>{
            private:
                InboundHalfEdgeIterable(const HalfEdge *edge) : Iterable<HalfEdge, InboundHalfEdgeIterator>(edge){}
                
            public:
                friend class Vertex;
            };
            
            InboundHalfEdgeIterable inbound() const{
                return edge;
            }
        };
        
        Vector2 vector2;
        HalfEdge *pair, *next;
        Face *mFace;
        Vertex *mVertex;
        
        HalfEdge() : pair(nullptr), next(nullptr), mFace(nullptr), mVertex(nullptr){}
        
        bool operator == (const HalfEdge &other) const{
            return vector2 == other.vector2 && pair == other.pair && next == other.next;
        }
        
        bool operator != (const HalfEdge &other) const{
            return !(*this == other);
        }
        
        bool operator < (const HalfEdge &other) const{
            if (vector2 < other.vector2){
                return true;
            }
            if (other.vector2 < vector2){
                return false;
            }
            if (pair < other.pair){
                return true;
            }
            if (other.pair < pair){
                return false;
            }
            return next < other.next;
        }
        
        const Face &face() const {
            return *mFace;
        }
        
        Face &face(){
            return *mFace;
        }
        
        const Vertex &vertex () const{
            return *mVertex;
        }
        
        Vertex &vertex(){
            return *mVertex;
        }
        
        Edge edge() const {
            return next ? Edge(vector2, next->vector2) : Edge(vector2, vector2);
        }
        
        HalfEdge *findLast() const{
            HalfEdge *last = next;
            while (last->next != this){
                last = last->next;
            }
            return last;
        }
        
        HalfEdge *nextPerimeterEdge(Set &visited) const{
            for (HalfEdge *i = next; i && i != this; i = i->next){
                if (i->pair == nullptr && visited.find(i) == visited.end()){
                    visited.insert(i);
                    return i;
                }
                else if (i->pair){
                    for (HalfEdge *j = i->pair->next; j && j != i->pair; ++j){
                        if (j->pair == nullptr && visited.find(j) == visited.end()){
                            visited.insert(j);
                            return j;
                        }
                    }
                }
            }
            return nullptr;
        }
        
        bool matches (const Edge &edge) const{
            return pair && ((vector2 == edge.endA && pair->vector2 == edge.endB) ||
            (vector2 == edge.endB && pair->vector2 == edge.endA));
        }
        
        bool fullyConnected() const{
            for (auto h : face().halfEdges()){
                if (!h.pair){
                    return false;
                }
            }
            return true;
        }
        
        bool onPerimeter() const{
            return !pair;
        }
        
        template <class Itr, class Allocator>
        static HalfEdge *fromPolygon (Itr begin, Itr end, Allocator &allocator, HalfEdge::Face *face){
            if (begin == end){
                return nullptr;
            }
            HalfEdge *first = allocator.allocate();
            face->edge = first;
            first->mFace = face;
            first->vector2 = *begin;
            HalfEdge *last = first;
            for (++begin; begin != end; ++begin){
                last->next = allocator.allocate();
                last->next->vector2 = *begin;
                last->next->mFace = face;
                last = last->next;
            }
            last->next = first;
            return first;
        }
        
        template<class Itr>
        static void Glue(Itr from, Itr to, HalfEdge::Vertex *vertex){
            vertex->edge = from->second;
            for (Itr i = from; i != to; ++i){
                i->second->mVertex = vertex;
                HalfEdge<FaceData, VertexData> *last = i->second->findLast();
                if (last->pair){
                    continue;
                }
                for (Itr j = from; j != to; ++j){
                    if (j->second->next->vector2 == last->vector2){
                        if (j->second->pair){
                            continue;
                        }
                        j->second->pair = last;
                        last->pair = j->second;
                        break;
                    }
                }
            }
        }
        
        class Builder
        {
        private:
            typedef std::multimap<Vector2, HalfEdge *> Fans;
            
            Fans fans;
         
        public:
            void add(HalfEdge *edge);
            
            template <class VertexAllocator>
            void construct(VertexAllocator &allocator){
                typename Fans::iterator start = fans.begin();
                for (auto i = fans.begin(); i != fans.end(); ++i){
                    if (start->first != i->first){
                        Glue(start, i, allocator.allocate());
                        start = i;
                    }
                }
                Glue(start, fans.end(), allocator.allocate());
            }
            
            friend std::ostream &operator<<(std::ostream &out, const Builder &builder){
                for (auto i : builder.fans){
                    std::cout << i.first << std::endl;
                }
                return out;
            }
        };
    };
    
    template<class FaceData, class VertexData>
    void HalfEdge<FaceData, VertexData>::Builder::add(HalfEdge *edge){
        HalfEdge *next = edge;
        do {
            fans.insert(std::make_pair(next->vector2, next));
            next = next->next;
        } while (next != edge);
    }
    
    template<class FaceData, class VertexData>
    Vector2 HalfEdge<FaceData, VertexData>::Face::calculateCentroid() const{
        Vector2 total(0.0f, 0.0f);
        float count = 0.0f;
        const HalfEdge *next = edge;
        do {
            total += next->vector2;
            next = next->next;
            count += 1.0f;
        } while (next != edge);
        return total / count;
    }

}

#endif /* half_edge_h */
