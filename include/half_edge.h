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
#include <set>
#include <map>
#include "optional.h"
#include <iostream>

namespace motu{
    
    struct EmptyData{
    };
    
    template<class VectorType, class FaceData = EmptyData, class VertexData = EmptyData>
    class HalfEdge
    {
    public:
        template <class T, class F, class V, class A>
        friend class VertexMap;
        
        typedef std::set<const HalfEdge*> Set;
        
        template<bool Const>
        struct HalfEdgeIteratorBase{
            typedef typename Optional<Const, const HalfEdge*, HalfEdge*>::value incremented_type;

            incremented_type first;
            bool incremented;
            
            HalfEdgeIteratorBase(incremented_type first, bool incremented) : first(first), incremented(incremented){}
            
            bool operator==(const HalfEdgeIteratorBase &other) const{
                return first == other.first && incremented == other.incremented;
            }
            
            bool operator!=(const HalfEdgeIteratorBase &other) const{
                return first && other.first && (first != other.first || incremented != other.incremented);
            }
        };
        
        template<class Type, class IteratorType, class ConstIteratorType>
        class Iterable{
        private:
            Type type;

        public:
            typedef IteratorType iterator;
            typedef ConstIteratorType const_iterator;
            
            Iterable(Type type) : type(type){}

            IteratorType begin() {
                return IteratorType(type, false);
            }
            
            IteratorType end() {
                return IteratorType(type, true);
            }
            
            ConstIteratorType begin() const{
                return ConstIteratorType(type, false);
            }
            
            ConstIteratorType end() const{
                return ConstIteratorType(type, true);
            }
        };

        class Face{
        public:
            friend class HalfEdge;
            
            template <class T, class F, class V, class A>
            friend class VertexMap;
            
            template<bool Const>
            struct BaseHalfEdgeIterator : public HalfEdgeIteratorBase<Const>{
                typedef typename Optional<Const, const HalfEdge*, HalfEdge*>::value pointer_type;
                typedef typename Optional<Const, const HalfEdge&, HalfEdge&>::value reference_type;
                
                BaseHalfEdgeIterator(typename HalfEdgeIteratorBase<Const>::incremented_type edge, bool incremented) : HalfEdgeIteratorBase<Const>(edge, incremented){}
                
                BaseHalfEdgeIterator &operator++(){
                    HalfEdgeIteratorBase<Const>::first = HalfEdgeIteratorBase<Const>::first->next;
                    HalfEdgeIteratorBase<Const>::incremented = true;
                    return *this;
                }
                
                BaseHalfEdgeIterator operator++(int){
                    BaseHalfEdgeIterator out(HalfEdgeIteratorBase<Const>::first, HalfEdgeIteratorBase<Const>::incremented);
                    HalfEdgeIteratorBase<Const>::first = HalfEdgeIteratorBase<Const>::first->next;
                    HalfEdgeIteratorBase<Const>::incremented = true;
                    return out;
                }
                
                reference_type operator*() {
                    return *HalfEdgeIteratorBase<Const>::first;
                }
                
                pointer_type operator->() {
                    return HalfEdgeIteratorBase<Const>::first;
                }
            };
            
            typedef BaseHalfEdgeIterator<false> HalfEdgeIterator;
            typedef BaseHalfEdgeIterator<true> ConstHalfEdgeIterator;
            typedef Iterable<typename HalfEdgeIterator::incremented_type, HalfEdgeIterator, ConstHalfEdgeIterator> HalfEdgeIterable;
            
        private:
            
            HalfEdge *edge;
            FaceData mData;
            
        public:
            typedef std::set<Face> Set;
            
            Face (HalfEdge *edge) : edge(edge){
            }
            
            Face() : edge(nullptr){}
            
            const HalfEdgeIterable halfEdges() const{
                return HalfEdgeIterable(edge);
            }
            
            HalfEdgeIterable halfEdges() {
                return HalfEdgeIterable(edge);
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
            
            VectorType calculateCentroid() const;
        };
        
        class Vertex{
        private:
            VectorType mPosition;
            HalfEdge *edge;
            VertexData mData;
            
        public:
            template <class T, class F, class V, class A>
            friend class VertexMap;
            
            Vertex (const VectorType &position, HalfEdge *edge) : mPosition(position), edge(edge){}
            
            Vertex () : edge(nullptr){}
            
            friend class HalfEdge;
            
            const VectorType &position() const{
                return mPosition;
            }
            
            VectorType &position(){
                return mPosition;
            }
            
            bool operator < (const Vertex &other) const{
                return mPosition < other.mPosition;
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
            
            template <bool Const>
            struct BaseInboundHalfEdgeIterator : public HalfEdgeIteratorBase<Const>{
                typedef typename Optional<Const, const HalfEdge &, HalfEdge &>::value reference_type;
                typedef typename Optional<Const, const HalfEdge *, HalfEdge *>::value pointer_type;
                
                BaseInboundHalfEdgeIterator(typename HalfEdgeIteratorBase<Const>::incremented_type edge, bool incremented) : HalfEdgeIteratorBase<Const>(edge, incremented){}
                
                BaseInboundHalfEdgeIterator &operator++(){
                    /*if (HalfEdgeIteratorBase<Const>::first->pair){
                        HalfEdgeIteratorBase<Const>::first = HalfEdgeIteratorBase<Const>::first->pair->next;
                    }
                    else{
                        HalfEdgeIteratorBase<Const>::first = nullptr;
                    }*/
                    HalfEdgeIteratorBase<Const>::first = HalfEdgeIteratorBase<Const>::first->pair->next;
                    HalfEdgeIteratorBase<Const>::incremented = true;
                    return *this;
                }
                
                BaseInboundHalfEdgeIterator operator++(int){
                    BaseInboundHalfEdgeIterator out(HalfEdgeIteratorBase<Const>::first, HalfEdgeIteratorBase<Const>::incremented);
                    ++*this;
                    return out;
                }
                
                reference_type operator*() {
                    return *HalfEdgeIteratorBase<Const>::first;
                }
                
                pointer_type operator->() {
                    return HalfEdgeIteratorBase<Const>::first;
                }

            };
            
            typedef BaseInboundHalfEdgeIterator<false> InboundHalfEdgeIterator;
            typedef BaseInboundHalfEdgeIterator<true> ConstInboundHalfEdgeIterator;
            
            typedef Iterable<typename InboundHalfEdgeIterator::incremented_type, InboundHalfEdgeIterator, ConstInboundHalfEdgeIterator> InboundHalfEdgeIterable;

            
            InboundHalfEdgeIterable inbound(){
                return edge;
            }
            
            const InboundHalfEdgeIterable inbound() const{
                return edge;
            }
            
            HalfEdge &halfEdge(){
                return *edge;
            }
            
            const HalfEdge &halfEdge() const{
                return *edge;
            }
            
            template<class HalfEdgeAllocator, class FaceAllocator>
            Face *erase(HalfEdgeAllocator &hAllocator, FaceAllocator &fAllocator){
                HalfEdge *floating = nullptr;
                Face *face = fAllocator.allocate();
                face->edge = edge->pair->next;
                std::vector<std::pair<HalfEdge*, HalfEdge*>> toConnect;
                std::vector<HalfEdge*> toDelete;
                for (auto i = inbound().begin(); i != inbound().end(); ++i){
                    toConnect.emplace_back(i->findLast(), i->pair->next);
                    toDelete.push_back(&*i);
                    toDelete.push_back(i->pair);
                    fAllocator.release(i->mFace);
                }
                for (auto i = toConnect.begin(); i != toConnect.end(); ++i){
                    i->first->next = i->second;
                    i->mFace = face;
                }
                for (auto i = toDelete.begin(); i != toDelete.end(); ++i){
                    hAllocator.release(*i);
                }
                return face;
            }
        };
        
        HalfEdge *pair, *next;
        Face *mFace;
        Vertex *mVertex;
        
        HalfEdge() : pair(nullptr), next(nullptr), mFace(nullptr), mVertex(nullptr){}
        
        bool operator == (const HalfEdge &other) const{
            return mVertex == other.mVertex && pair == other.pair;
        }
        
        bool operator != (const HalfEdge &other) const{
            return !(*this == other);
        }
        
        bool operator < (const HalfEdge &other) const{
            if (mVertex < other.mVertex){
                return true;
            }
            if (other.mVertex < mVertex){
                return false;
            }
            return other.pair < pair;
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
        
        template<class HalfEdgeAllocator>
        void split(Vertex *vertex, HalfEdgeAllocator &allocator){
            vertex->mPosition = mVertex->mPosition + ((next->mVertex->mPosition - mVertex->mPosition) * 0.5f);
            HalfEdge *a = allocator.allocate();
            vertex->edge = a;
            a->mVertex = vertex;
            a->next = next;
            a->mFace = mFace;
            next = a;
            if (pair){
                HalfEdge *b = allocator.allocate();
                b->mVertex = vertex;
                b->next = pair->next;
                pair->next = b;
                b->mFace = pair->mFace;
                a->pair = b;
                b->pair = a;
            }
        }
        
        HalfEdge *findLast() const{
            HalfEdge *last = next;
            while (last->next != this){
                last = last->next;
            }
            return last;
        }
        
        template <class Itr, class VertexMap, class Allocator>
        static HalfEdge *fromPolygon (Itr begin, Itr end, VertexMap &vertexMap, Allocator &allocator, typename HalfEdge::Face *face){
            if (begin == end){
                return nullptr;
            }
            HalfEdge *first = allocator.allocate();
            face->edge = first;
            first->mFace = face;
            first->mVertex = vertexMap.get(*begin, first);
            HalfEdge *last = first;
            for (++begin; begin != end; ++begin){
                last->next = allocator.allocate();
                last->mVertex = vertexMap.get(*begin, last);
                last->next->mFace = face;
                last = last->next;
            }
            last->next = first;
            return first;
        }
        
        
        template <class IteratorType>
        static IteratorType findHalfEdgeConnecting(IteratorType begin, IteratorType end, const Vertex &vertex){
            const Vertex *ptr = &vertex;
            while (begin != end){
                if (&begin->vertex() == ptr || &begin->next->vertex() == ptr){
                    return begin;
                }
                ++begin;
            }
            return end;
        }
    };
    
    template<class VectorType, class FaceData, class VertexData>
    VectorType HalfEdge<VectorType, FaceData, VertexData>::Face::calculateCentroid() const{
        VectorType total;
        total.zero();
        float count = 0.0f;
        const HalfEdge *next = edge;
        do {
            total += next->vertex().position();
            next = next->next;
            count += 1.0f;
        } while (next != edge);
        return total / count;
    }
    
    template <class VectorType, class EdgeAllocator, class VertexAllocator, class FaceAllocator>
    class VertexMap{
    private:
        typedef typename EdgeAllocator::ObjectType HalfEdge;
        typedef typename HalfEdge::Vertex Vertex;
        typedef typename HalfEdge::Face Face;
        typedef std::map<VectorType, Vertex *> Map;
        typedef std::multimap<Vertex *, HalfEdge *> Fans;
        
        Map map;
        EdgeAllocator *edgeAllocator;
        VertexAllocator *vertexAllocator;
        FaceAllocator *faceAllocator;
        
    public:
        VertexMap(EdgeAllocator *edgeAllocator, VertexAllocator *vertexAllocator, FaceAllocator *faceAllocator) : edgeAllocator(edgeAllocator), vertexAllocator(vertexAllocator), faceAllocator(faceAllocator){}
        
        void set(const VectorType &position, HalfEdge *halfEdge){
            auto i = map.find(position);
            if (i == map.end()){
                i = map.insert(i, std::make_pair(position, vertexAllocator->allocate()));
                i->second->mPosition = position;
                i->second->edge = halfEdge;
            }
            halfEdge->mVertex = i->second;
        }
        
        static void Glue(typename Fans::iterator from, typename Fans::iterator to){
            for (auto i = from; i != to; ++i){
                HalfEdge *last = i->second->findLast();
                if (last->pair){
                    continue;
                }
                for (auto j = from; j != to; ++j){
                    if ((!j->second->pair) && &j->second->next->vertex() == &last->vertex()){
                        j->second->pair = last;
                        last->pair = j->second;
                        break;
                    }
                }
            }
        }
        
        Face *createExternalFace(){
            Face *face = faceAllocator->allocate();
            std::vector<HalfEdge *> externalEdges;
            for (auto i = edgeAllocator->begin(); i != edgeAllocator->end(); ++i){
                if (!i->pair){
                    externalEdges.emplace_back(&*i);
                }
            }
            std::cout << externalEdges.size() << " external edges" << std::endl;
            std::multimap<VectorType, HalfEdge *> outsideEdges;
            for (auto i = externalEdges.begin(); i != externalEdges.end(); ++i){
                (*i)->pair = edgeAllocator->allocate();
                (*i)->pair->pair = *i;
                (*i)->pair->mVertex = (*i)->next->mVertex;
                outsideEdges.emplace((*i)->pair->mVertex->position(), (*i)->pair);
            }
            for (auto i = externalEdges.begin(); i != externalEdges.end(); ++i){
                auto j = outsideEdges.equal_range((*i)->vertex().position());
                if (j.first != j.second){
                    (*i)->pair->next = j.first->second;
                    outsideEdges.erase(j.first);
                }
                else{
                    std::cout << "Missing vertex" << std::endl;
                    exit(1);
                }
                /*if (outsideEdges.find((*i)->vertex().position()) == outsideEdges.end()){
                    std::cout << "Missing vertex" << std::endl;
                    exit(1);
                }
                (*i)->pair->next = outsideEdges.find((*i)->vertex().position())->second;*/
                (*i)->pair->mFace = face;
                face->edge = (*i)->pair;
            }
            //----------------
            std::cout << "Checking external face" << std::endl;
            int count = 0;
            for (auto i = face->halfEdges().begin(); i != face->halfEdges().end(); ++i){
                ++count;
            }
            if (count != externalEdges.size()){
                std::cout << "Bugger " << count << std::endl;
                //exit(1);
            }
            std::cout << "Found " << count << " external edges" << std::endl;
            //-------------------
			if (face->halfEdge() == nullptr) {
				exit(1);
			}
            return face;
        }
        
        Face *bind(){
            Fans fans;
            for (auto i = edgeAllocator->begin(); i != edgeAllocator->end(); ++i){
                fans.insert(std::make_pair(&i->vertex(), &*i));
            }
            typename Fans::iterator start = fans.begin();
            for (auto i = fans.begin(); i != fans.end(); ++i){
                if (start->first != i->first){
                    Glue(start, i);
                    start = i;
                }
            }
            Glue(start, fans.end());
            return createExternalFace();
        }
        
        template <class Itr>
        void addPolygon (Itr begin, Itr end){
            if (begin == end){
                return;
            }
            Face *face = faceAllocator->allocate();
            HalfEdge *first = edgeAllocator->allocate();
            face->edge = first;
            first->mFace = face;
            set(*begin, first);
            HalfEdge *last = first;
            for (++begin; begin != end; ++begin){
                last->next = edgeAllocator->allocate();
                set(*begin, last->next);
                last->next->mFace = face;
                last = last->next;
            }
            last->next = first;
        }
    };
}

#endif /* half_edge_h */
