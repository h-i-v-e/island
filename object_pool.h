//
//  object_pool.h
//  World Maker
//
//  Created by Jerome Johnson on 15/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef object_pool_h
#define object_pool_h

#include <cstddef>
#include <cstdint>
#include <utility>

namespace worldmaker{
    template<class ObjectType>
    class ObjectPool{
    private:
        size_t blockSize;
        
        union PoolItem{
            uint8_t memory[sizeof(ObjectType)];
            PoolItem *nextFree;
        };
        
        struct AllocationBlock{
            PoolItem *items;
            AllocationBlock *nextBlock;
            
            AllocationBlock(size_t blockSize) : nextBlock(nullptr), items(new PoolItem[blockSize]) {
                PoolItem *last = items + blockSize - 1;
                last->nextFree = nullptr;
                for (PoolItem *i = items; i != last; ++i){
                    i->nextFree = i + 1;
                }
            }
            
            ~AllocationBlock(){
                delete [] items;
            }
        };
        
        PoolItem *nextFree;
        AllocationBlock *firstBlock;
        
        void *_allocate(){
            PoolItem *ret = nextFree;
            if (!ret){
                AllocationBlock *newBlock = new AllocationBlock(blockSize);
                newBlock->nextBlock = firstBlock;
                firstBlock = newBlock;
                ret = newBlock->items;
            }
            nextFree = ret->nextFree;
            return ret->memory;
        }

        
    public:
        ObjectPool(size_t blockSize) : blockSize(blockSize), firstBlock(new AllocationBlock(blockSize)){
            nextFree = firstBlock->items;
        }
        
        ~ObjectPool(){
            while(firstBlock){
                AllocationBlock *nextBlock = firstBlock->nextBlock;
                delete firstBlock;
                firstBlock = nextBlock;
            }
        }
        
        template <typename... Args>
        ObjectType *allocate(Args&&... args){
            return new(_allocate())ObjectType(std::forward<Args>(args)...);
        }
        
        void release(ObjectType *data){
            PoolItem *poolItem = reinterpret_cast<PoolItem*>(data);
            poolItem->nextFree = nextFree;
            nextFree = poolItem;
        }
    };
}

#endif /* object_pool_h */
