#ifndef UTIL_H
#define UTIL_H

#include <cfloat>
#include <cstdint>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>

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

	template<class T>
	void writeOutVector(std::ostream &out, const std::vector<T> &vec) {
		size_t size = vec.size();
		out.write(reinterpret_cast<const char *>(&size), sizeof(size_t));
		out.write(reinterpret_cast<const char *>(vec.data()), size * sizeof(T));
	}

	template<class T>
	void readInVector(std::istream &in, std::vector<T> &vec) {
		size_t size;
		in.read(reinterpret_cast<char*>(&size), sizeof(size_t));
		vec.resize(size);
		in.read(reinterpret_cast<char*>(vec.data()), size * sizeof(T));
	}

	template<class T>
	void writeOutSet(std::ostream &out, const std::unordered_set<T> &set) {
		std::vector<T> tmp;
		tmp.reserve(set.size());
		std::copy(set.begin(), set.end(), std::back_inserter(tmp));
		writeOutVector(out, tmp);
	}

	template<class T>
	void readInSet(std::istream &in, std::unordered_set<T> &set) {
		std::vector<T> tmp;
		readInVector(in, tmp);
		set.reserve(tmp.size());
		for (const auto &i : tmp) {
			set.insert(i);
		}
	}

	template<class K, class V>
	void writeOutMap(std::ostream &out, const std::unordered_map<K, V> &map) {
		std::vector<std::pair<K, V>> tmp;
		tmp.reserve(map.size());
		std::copy(map.begin(), map.end(), std::back_inserter(tmp));
		writeOutVector(out, tmp);
	}

	template<class K, class V>
	void readInMap(std::istream &in, std::unordered_map<K, V> &map) {
		std::vector<std::pair<K, V>> tmp;
		readInVector(in, tmp);
		map.reserve(tmp.size());
		for (const auto &i : tmp) {
			map.insert(i);
		}
	}
}
#endif