#include "height_map.h"
#include "mesh.h"

using namespace motu;

namespace {
	template <class GetMaxHeight>
	float LoadHeightMap(HeightMap &heightMap, const Mesh &mesh, GetMaxHeight getMaxHeight) {
		float max = std::numeric_limits<float>::min(), min = std::numeric_limits<float>::max();
		for (const Vector3 &vert : mesh.vertices) {
			if (vert.z < min) {
				min = vert.z;
			}
			if (vert.z > max) {
				max = vert.z;
			}
		}
		float mul = getMaxHeight(min, max) / max - min;
		size_t length = heightMap.width() * heightMap.height();
		const float *end = heightMap.data() + length;
		for (float *i = heightMap.data(); i != end; ++i) {
			*i = min;
		}
		mesh.rasterize(heightMap);
		for (float *i = heightMap.data(); i != end; ++i) {
			*i = (*i - min) * mul;
		}
		return min * -mul;
	}
}

void HeightMap::load(const Mesh &mesh, float maxHeight) {
	mSeaLevel = LoadHeightMap(*this, mesh, [maxHeight](float min, float max) {
		return maxHeight;
	});
}

void HeightMap::load(const Mesh &mesh) {
	mSeaLevel = LoadHeightMap(*this, mesh, [](float min, float max) {
		float val = max - min;
		return val > 1.0f ? 1.0f : val;
	});
}


