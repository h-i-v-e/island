#ifndef iterable_releasable_object_pool_h
#define iterable_releasable_object_pool_h

#include "object_pool.h"

namespace motu {
	template <class Type>
	class IterableReleasableObjectPool {
	private:
		struct PoolItem {
			Type type;
			PoolItem *next, *last;
		};

		ObjectPool<PoolItem> pool;
		PoolItem *lastAllocated;
		size_t items;
	public:
		typedef Type ObjectType;

		IterableReleasableObjectPool(size_t size) : pool(size), lastAllocated(nullptr), items(0) {}

		Type *allocate() {
			PoolItem *item = pool.allocate();
			item->next = nullptr;
			if (lastAllocated) {
				lastAllocated->next = item;
			}
			item->last = lastAllocated;
			lastAllocated = item;
			++items;
			return &item->type;
		}

		void drain() {
			while (lastAllocated) {
				PoolItem *next = lastAllocated->last;
				pool.release(lastAllocated);
				lastAllocated = next;
			}
			items = 0;
		}

		void release(Type *type) {
			PoolItem *item = reinterpret_cast<PoolItem*>(type);
			PoolItem *next = item->next;
			if (next) {
				next->last = item->last;
			}
			if (item->last) {
				item->last->next = next;
			}
			else {
				lastAllocated = next;
			}
			pool.release(item);
		}

		class iterator {
		protected:
			PoolItem *item;
		public:
			iterator(PoolItem *item) : item(item) {}

			iterator &operator++() {
				item = item->last;
				return *this;
			}

			iterator operator++(int) {
				iterator out(item);
				item = item->last;
				return out;
			}

			Type &operator*() {
				return item->type;
			}

			Type *operator->() {
				return &item->type;
			}

			bool operator==(const iterator &other) const {
				return item == other.item;
			}

			bool operator!=(const iterator &other) const {
				return item != other.item;
			}
		};

		class const_iterator {
		protected:
			const PoolItem *item;
		public:
			const_iterator(const PoolItem *item) : item(item) {}

			const_iterator &operator++() {
				item = item->last;
				return *this;
			}

			const_iterator operator++(int) {
				const_iterator out(item);
				item = item->last;
				return out;
			}

			const Type &operator*() {
				return item->type;
			}

			const Type *operator->() {
				return &item->type;
			}

			bool operator==(const const_iterator &other) const {
				return item == other.item;
			}

			bool operator!=(const const_iterator &other) const {
				return item != other.item;
			}
		};

		iterator begin() {
			return lastAllocated;
		}

		iterator end() {
			return nullptr;
		}

		const_iterator begin() const {
			return lastAllocated;
		}

		const_iterator end() const {
			return nullptr;
		}

		size_t size() const {
			return items;
		}
	};
}

#endif