//
//  half_edge_pool.h
//  World Maker
//
//  Created by Jerome Johnson on 25/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef half_edge_pool_h
#define half_edge_pool_h

#include "object_pool.h"

namespace worldmaker{
    template <class HalfEdge>
    class HalfEdgePool{
    private:
        struct PoolItem{
            HalfEdge halfEdge;
            PoolItem *previousAllocated;
        };
        
        ObjectPool<PoolItem> pool;
        
        PoolItem *lastAllocated;
        
    public:
        HalfEdgePool(size_t allocationBlocks) : pool(allocationBlocks), lastAllocated(nullptr){}

        HalfEdge *allocate(){
            PoolItem *item = pool.allocate();
            item->previousAllocated = lastAllocated;
            lastAllocated = item;
            return &item->halfEdge;
        }
        
        void drain(){
            while (lastAllocated){
                PoolItem *next = lastAllocated->previousAllocated;
                pool.release(lastAllocated);
                lastAllocated = next;
            }
        }
        
        class const_iterator{
        protected:
            const PoolItem *item;
        public:
            const_iterator(const PoolItem *item) : item(item){}
            
            const_iterator &operator++(){
                item = item->previousAllocated;
                return *this;
            }
            
            const_iterator operator++(int){
                const_iterator out(item);
                item = item->previousAllocated;
                return out;
            }
            
            const HalfEdge &operator*(){
                return item->halfEdge;
            }
            
            const HalfEdge *operator->(){
                return &item->halfEdge;
            }
            
            bool operator==(const const_iterator &other) const{
                return item == other.item;
            }
            
            bool operator!=(const const_iterator &other) const{
                return item != other.item;
            }
        };
        
        const_iterator begin() const{
            return lastAllocated;
        }
        
        const_iterator end() const{
            return nullptr;
        }
    };
}

#endif /* half_edge_pool_h */
