//
//  half_edge_pool.h
//  World Maker
//
//  Created by Jerome Johnson on 25/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef iterable_object_pool_h
#define iterable_object_pool_h

#include "object_pool.h"

namespace worldmaker{
    template <class Type>
    class IterableObjectPool{
    private:
        struct PoolItem{
            Type type;
            PoolItem *previousAllocated;
        };
        
        ObjectPool<PoolItem> pool;
        
        PoolItem *lastAllocated;
        
    public:
        IterableObjectPool(size_t allocationBlocks) : pool(allocationBlocks), lastAllocated(nullptr){}

        Type *allocate(){
            PoolItem *item = pool.allocate();
            item->previousAllocated = lastAllocated;
            lastAllocated = item;
            return &item->type;
        }
        
        void drain(){
            while (lastAllocated){
                PoolItem *next = lastAllocated->previousAllocated;
                pool.release(lastAllocated);
                lastAllocated = next;
            }
        }
        
        class iterator{
        protected:
            PoolItem *item;
        public:
            iterator(PoolItem *item) : item(item){}
            
            iterator &operator++(){
                item = item->previousAllocated;
                return *this;
            }
            
            iterator operator++(int){
                iterator out(item);
                item = item->previousAllocated;
                return out;
            }
            
            Type &operator*(){
                return item->type;
            }
            
            Type *operator->(){
                return &item->type;
            }
            
            bool operator==(const iterator &other) const{
                return item == other.item;
            }
            
            bool operator!=(const iterator &other) const{
                return item != other.item;
            }
        };
        
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
            
            const Type &operator*(){
                return item->type;
            }
            
            const Type *operator->(){
                return &item->type;
            }
            
            bool operator==(const const_iterator &other) const{
                return item == other.item;
            }
            
            bool operator!=(const const_iterator &other) const{
                return item != other.item;
            }
        };
        
        iterator begin(){
            return lastAllocated;
        }
        
        iterator end(){
            return nullptr;
        }
        
        const_iterator begin() const{
            return lastAllocated;
        }
        
        const_iterator end() const{
            return nullptr;
        }
    };
}

#endif /* iterable_object_pool_h */
