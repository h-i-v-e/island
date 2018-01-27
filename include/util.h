#ifndef UTIL_H
#define UTIL_H

#define HASH_PRIME_A 1610612741
#define HASH_PRIME_B 805306457
#define HASH_PRIME_C 402653189
#define HASH_PRIME_D 201326611

namespace motu{
	template<class T>
	struct Hasher {
		size_t operator()(const T &t) const {
			return t.hash();
		}
	};

	inline bool almostEqual(float a, float b) {
		float test = a - b;
		return test >= -FLT_EPSILON && test <= FLT_EPSILON;
	}

	inline bool almostZero(float f) {
		return f >= -FLT_EPSILON && f <= FLT_EPSILON;
	}

	static size_t hashFloat(float f) {
		return *reinterpret_cast<uint32_t*>(&f);
	}
}
#endif