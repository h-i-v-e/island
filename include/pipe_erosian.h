#ifndef MOTU_PIPE_EROSIAN
#define MOTU_PIPE_EROSIAN

#include <random>

namespace motu {

	class HeightMap;

	class PipeErosian {
	public:
		PipeErosian(std::default_random_engine &rd, HeightMap &heightMap) : rd(rd), heightMap(heightMap) {}

		void erode();
	private:
		std::default_random_engine &rd;
		HeightMap &heightMap;
	};
}

#endif