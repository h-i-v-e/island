#include "util.h"

#ifndef MOTU_VECTOR2_INT
#define MOTU_VECTOR2_INT

namespace motu {
	struct Vector2Int {
		int x, y;

		Vector2Int() {}

		Vector2Int(int x, int y) : x(x), y(y) {}

		size_t hash() const {
			return x * HASH_PRIME_A * y * HASH_PRIME_B;
		}

		bool operator==(const Vector2Int &other) const {
			return x == other.x && y == other.y;
		}

		bool operator!=(const Vector2Int &other) const {
			return x != other.x && y != other.y;
		}
	};
}

#endif