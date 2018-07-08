#include "height_map.h"
#include "mesh.h"
#include "vector2int.h"

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

	float interpTrBl(const HeightMap &hm, float x, float y) {
		Vector2Int tr(x, y);
		Vector2Int bl(tr.x, tr.y + 1);
		++tr.x;
		if (tr.x == hm.width()) {
			--tr.x;
		}
		if (bl.y == hm.height()) {
			--bl.y;
		}
		float xOff = x - bl.x;
		float yOff = y - tr.y;
		if (yOff > xOff) {
			float trv = hm(tr.x, tr.y);
			float top = (trv * xOff) + (hm(bl.x, tr.y) * (1.0f - xOff));
			float right = (trv * yOff) + (hm(tr.x, bl.y) * (1.0f - yOff));
			return (top + right) * 0.5f;
		}
		else {
			float blv = hm(bl.x, bl.y);
			float bottom = (blv * (1.0f - xOff)) + (hm(tr.x, bl.y) * xOff);
			float left = (blv * (1.0f - yOff)) + (hm(bl.x, tr.y) * yOff);
			return (bottom + left) * 0.5f;
		}
	}

	float interpTlBr(const HeightMap &hm, float x, float y) {
		Vector2Int tl(x, y);
		Vector2Int br(tl.x + 1, tl.y + 1);
		if (br.x == hm.width()) {
			--br.x;
		}
		if (br.y == hm.height()) {
			--br.y;
		}
		float xOff = x - tl.x;
		float yOff = y - tl.y;
		if (yOff < FLT_EPSILON || (xOff / yOff) < 0.5f) {
			float tlv = hm(tl.x, tl.y);
			float top = (tlv * (1.0f - xOff)) + (hm(br.x, tl.y) * xOff);
			float left = (tlv * (1.0f - yOff)) + (hm(tl.x, br.y) * yOff);
			return (left + top) * 0.5f;
		}
		else {
			float brv = hm(br.x, br.y);
			float bottom = (brv * xOff) + (hm(tl.x, br.y) * (1.0f - xOff));
			float right = (brv * yOff) + (hm(br.x, tl.y) * (1.0f - yOff));
			return (bottom + right) * 0.5f;
		}
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


void HeightMap::smooth() {
	static float diagWeight = 1.0f / sqrtf(2.0f);
	static float mul = 1.0f / ((diagWeight * 4.0f) + 5.0f);
	Grid<float> buffer(width(), height());
	for (int y = 1, hLast = height() - 1, xLast = width() - 1; y < hLast; ++y) {
		for (int x = 1; x < xLast; ++x) {
			float total = operator()(x, y);
			if (total <= mSeaLevel) {
				buffer(x, y) = total;
				continue;
			}
			total += operator()(x - 1, y - 1) * diagWeight;
			total += operator()(x, y - 1);
			total += operator()(x + 1, y - 1) * diagWeight;
			total += operator()(x - 1, y);
			total += operator()(x + 1, y);
			total += operator()(x - 1, y + 1) * diagWeight;
			total += operator()(x, y + 1);
			total += operator()(x + 1, y + 1) * diagWeight;
			buffer(x, y) = total * mul;
		}
	}
	std::copy(buffer.data(), buffer.data() + width() * height(), data());
}


float HeightMap::interpTRBL(float x, float y) {
	Vector2Int tr(x, y);
	Vector2Int bl(tr.x, tr.y + 1);
	++tr.x;
	if (tr.x == width()) {
		--tr.x;
	}
	if (bl.y == height()) {
		--bl.y;
	}
	float xOff = x - bl.x;
	float yOff = y - tr.y;
	if (yOff > xOff) {
		float trv = operator()(tr.x, tr.y);
		float top = (trv * xOff) + (operator()(bl.x, tr.y) * (1.0f - xOff));
		float right = (trv * yOff) + (operator()(tr.x, bl.y) * (1.0f - yOff));
		return (top + right) * 0.5f;
	}
	else {
		float blv = operator()(bl.x, bl.y);
		float bottom = (blv * (1.0f - xOff)) + (operator()(tr.x, bl.y) * xOff);
		float left = (blv * (1.0f - yOff)) + (operator()(bl.x, tr.y) * yOff);
		return (bottom + left) * 0.5f;
	}
}

float HeightMap::interpTLBR(float x, float y) {
	Vector2Int tl(x, y);
	Vector2Int br(tl.x + 1, tl.y + 1);
	if (br.x == width()) {
		--br.x;
	}
	if (br.y == height()) {
		--br.y;
	}
	float xOff = x - tl.x;
	float yOff = y - tl.y;
	if (xOff < yOff) {
		float tlv = operator()(tl.x, tl.y);
		float top = (tlv * (1.0f - xOff)) + (operator()(br.x, tl.y) * xOff);
		float left = (tlv * (1.0f - yOff)) + (operator()(tl.x, br.y) * yOff);
		return (left + top) * 0.5f;
	}
	else {
		float brv = operator()(br.x, br.y);
		float bottom = (brv * xOff) + (operator()(tl.x, br.y) * (1.0f - xOff));
		float right = (brv * yOff) + (operator()(br.x, tl.y) * (1.0f - yOff));
		return (bottom + right) * 0.5f;
	}
}

void HeightMap::raise(const Vector3 &v3) {
	Vector2Int tl(v3.x, v3.y);
	Vector2Int br(tl.x + 1, tl.y + 1);
	if (br.x == width()) {
		--br.x;
	}
	if (br.y == height()) {
		--br.y;
	}
	Vector2Int coords[4] = {
		tl,
		Vector2Int(br.x, tl.y),
		Vector2Int(tl.x, br.y),
		br
	};
	float distances[4];
	float total = 0.0f;
	for (size_t i = 0; i != 4; ++i) {
		float mag = (Vector2(coords[i].x, coords[i].y) - v3.asVector2()).magnitude();
		distances[i] = mag;
		total += mag;
	}
	float out = 0.0f;
	for (size_t i = 0; i != 4; ++i) {
		out += (total - distances[i]) * operator()(coords[i].x, coords[i].y);
	}
	float shift = v3.z - (out / total);
	for (size_t i = 0; i != 4; ++i) {
		operator()(coords[i].x, coords[i].y) += shift;
	}
}


