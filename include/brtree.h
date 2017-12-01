//
//  brtree.h
//  World Maker
//
//  Created by Jerome Johnson on 8/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef brtree_h
#define brtree_h

#include "bounding_rectangle.h"
#include "object_pool.h"
#include <vector>

namespace motu{
    template<class ValueType>
    class BRTree{
    public:
        typedef std::vector<const ValueType*> Values;
        
        class const_iterator{
        private:
            typename Values::const_iterator itr;
        public:
            const_iterator(typename Values::const_iterator itr) : itr(itr){
            }
            
            const ValueType &operator*() const{
                return **itr;
            }
            
            const ValueType *operator->() const{
                return *itr;
            }
            
            const_iterator &operator++(){
                ++itr;
                return *this;
            }
            
            const_iterator operator++(int){
                return const_iterator(itr++);
            }
            
            bool operator==(const const_iterator &other) const{
                return itr == other.itr;
            }
            
            bool operator!=(const const_iterator &other) const{
                return itr != other.itr;
            }
        };
        
    private:
        struct Node;
        
        inline Node *allocate(const BoundingRectangle &bounds, const ValueType &value, Node *parent);
        
        inline void release(Node *node);
        
        struct Node{
            Node *left, *right, *parent;
            BoundingRectangle totalBounds, valueBounds;
            ValueType value;
            
            Node():left(nullptr), right(nullptr), parent(nullptr){
                totalBounds.clear();
                valueBounds.clear();
            }
            
            Node(const BoundingRectangle &bounds, const ValueType &value, Node *parent) :
            left(nullptr), right(nullptr),  parent(parent), totalBounds(bounds), valueBounds(bounds), value(value){}
            
            Node *createChild(BRTree &tree, const BoundingRectangle &bounds, const ValueType &value){
                if (bounds.area() > valueBounds.area()){
                    Node *node = tree.allocate(valueBounds, this->value, this);
                    valueBounds = bounds;
                    this->value = value;
                    return node;
                }
                else{
                    return tree.allocate(bounds, value, this);
                }
            }
            
            void addBounds(const Node *node){
                if (node){
                    totalBounds += node->totalBounds;
                }
            }
            
            void recomputeBounds(){
                totalBounds.clear();
                addBounds(left);
                addBounds(right);
                if (!valueBounds.empty()){
                    totalBounds += valueBounds;
                }
            }
            
            void intersecting(const BoundingRectangle &bounds, Values &values) const{
                if ((!valueBounds.empty()) && valueBounds.intersects(bounds)){
                    values.push_back(&value);
                }
                if (left && left->totalBounds.intersects(bounds)){
                    left->intersecting(bounds, values);
                }
                if (right && right->totalBounds.intersects(bounds)){
                    right->intersecting(bounds, values);
                }
            }
            
            void containing(const Vector2 &pos, Values &values) const{
                if ((!valueBounds.empty()) && valueBounds.contains(pos)){
                    values.push_back(&value);
                }
                if (left && left->totalBounds.contains(pos)){
                    left->containing(pos, values);
                }
                if (right && right->totalBounds.contains(pos)){
                    right->containing(pos, values);
                }
            }
            
            void all(Values &values) const{
                if (!valueBounds.empty()){
                    values.push_back(&value);
                }
                if (left){
                    left->all(values);
                }
                if (right){
                    right->all(values);
                }
            }
            
            int getDepth(int depth) const{
                int deepest = ++depth;
                if (left){
                    deepest = left->getDepth(depth);
                }
                if (right){
                    int rDepth = right->getDepth(depth);
                    if (rDepth > deepest){
                        deepest = rDepth;
                    }
                }
                return deepest;
            }
            
            void addDown(BRTree &tree, const BoundingRectangle &bounds, const ValueType &value){
                if (!left){
                    left = tree.allocate(bounds, value, this);
                }
                else if (!right){
                    right = tree.allocate(bounds, value, this);
                }
                else{
                    float leftArea = (left->totalBounds + bounds).area();
                    float rightArea = (right->totalBounds + bounds).area();
                    float leftGrowth = leftArea - left->totalBounds.area();
                    float rightGrowth = rightArea - right->totalBounds.area();
                    if (leftGrowth < rightGrowth){
                        left->add(tree, bounds, value);
                    }
                    else if (rightGrowth < leftGrowth){
                        right->add(tree, bounds, value);
                    }
                    else if (leftArea < rightArea){
                        left->add(tree, bounds, value);
                    }
                    else{
                        right->add(tree, bounds, value);
                    }
                }
            }
            
            void add(BRTree &tree, const BoundingRectangle &bounds, const ValueType &value){
                if (valueBounds.empty()){
                    totalBounds = valueBounds = bounds;
                    this->value = value;
                }
                else{
                    totalBounds += bounds;
                    if (bounds.area() > valueBounds.area()){
                        addDown(tree, valueBounds, this->value);
                        valueBounds = bounds;
                        this->value = value;
                    }
                    else{
                        addDown(tree, bounds, value);
                    }
                }
            }
            
            void swapUp(Node *child, BRTree &tree){
                valueBounds = child->valueBounds;
                value = child->value;
                child->remove(tree, valueBounds, value);
                recomputeBounds();
            }
            
            void removeChildNode(BRTree &tree, Node *node){
                if (node == left){
                    left = nullptr;
                }
                else{
                    right = nullptr;
                }
                tree.release(node);
                recomputeBounds();
            }
            
            bool remove(BRTree &tree, const BoundingRectangle &bounds, const ValueType &value){
                if (this->value == value){
                    if (left){
                        if (right && right->valueBounds.area() > left->valueBounds.area()){
                            swapUp(right, tree);
                        }
                        else{
                            swapUp(left, tree);
                        }
                    }
                    else if (right){
                        swapUp(right, tree);
                    }
                    else if (parent){
                        parent->removeChildNode(tree, this);
                    }
                    else{
                        valueBounds.clear();
                        totalBounds.clear();
                    }
                    return true;
                }
                else if (
                         (left && left->totalBounds.intersects(bounds) && left->remove(tree, bounds, value)) ||
                         (right && right->totalBounds.intersects(bounds) && right->remove(tree, bounds, value))
                         ){
                    recomputeBounds();
                    return true;
                }
                return false;
            }
        };
        
        ObjectPool<Node> objectPool;
        
        typedef std::vector<Node> Nodes;
        
        Node *root;
    public:
        
        BRTree(size_t allocationBlockSize) : objectPool(allocationBlockSize){
            root = objectPool.allocate();
        }
        
        void add(const BoundingRectangle &bounds, const ValueType &value){
            root->add(*this, bounds, value);
        }
        
        bool remove(const BoundingRectangle &bounds, const ValueType &value){
            return root->remove(*this, bounds, value);
        }
        
        int getDepth() const{
            return root->getDepth(0);
        }
                                
        std::pair<const_iterator, const_iterator> intersecting(const BoundingRectangle &bounds, Values &buf) const{
            buf.clear();
            root->intersecting(bounds, buf);
            return std::make_pair(const_iterator(buf.begin()), const_iterator(buf.end()));
        }
        
        std::pair<const_iterator, const_iterator> containing(const Vector2 &pos, Values &buf) const{
            buf.clear();
            root->containing(pos, buf);
            return std::make_pair(const_iterator(buf.begin()), const_iterator(buf.end()));
        }
        
        std::pair<const_iterator, const_iterator> all(Values &buf) const{
            buf.clear();
            root->all(buf);
            return std::make_pair(const_iterator(buf.begin()), const_iterator(buf.end()));
        }
    };
    
    template<class ValueType>
    typename BRTree<ValueType>::Node *BRTree<ValueType>::allocate(const BoundingRectangle &bounds, const ValueType &value, Node *parent){
        return objectPool.allocate(bounds, value, parent);
    }
    
    template<class ValueType>
    void BRTree<ValueType>::release(Node *node){
        objectPool.release(node);
    }
}

#endif /* brtree_h */
