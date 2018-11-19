#include "texture_utils.h"

#include "mesh.h"

using namespace motu;

namespace {
	short occlusianCursor[30] = {
		-8, -2, -4, 2, -2, 1, -1, 1,
		-1, -1, -1, -4, 1, 2, 1, 1,
		1, -1, 1, -2, 2, 4, 2, -1,
		2, -8, 4, -2, 8, 2
	};

	uint8_t computeOcclusian(const Grid<Mesh::VertexAndNormal> &grid, int offset) {
		float total = 0.0f;
		size_t count = 0;
		int width = grid.width(), height = grid.height();
		const Mesh::VertexAndNormal &van = grid.data()[offset];
		int x = offset % width, y = offset / width;
		for (size_t i = 0; i != 30; i += 2) {
			int px = x + occlusianCursor[i];
			int py = y + occlusianCursor[i + 1];
			if (px >= 0 && px < grid.width() && py >= 0 && py < grid.height()) {
				Vector3 dir(grid(px, py).vertex - van.vertex);
				if (dir.squareMagnitude() < FLT_EPSILON) {
					continue;
				}
				float raw = dir.normalize().dot(van.normal);
				total += raw < 0.0f ? 0.0f : raw;
				++count;
			}
		}
		return count > 0 ? static_cast<uint8_t>((1.0f - (total / count)) * 255.0f) : 0xff;
	}

	uint32_t clampToU8(float in) {
		if (in < 0.0f) {
			return 0;
		}
		if (in > 255.0f) {
			return 255;
		}
		return static_cast<uint32_t>(in);
	}

	/*uint32_t createNormalValue(const Vector3 &vec) {
		uint32_t out = clampToU8(127.5f + (vec.x * 127.5f)) << 8;
		out |= clampToU8(127.5f + (vec.y * 127.5f)) << 16;
		return out | (clampToU8(127.5f + (vec.z * 127.5f)) << 24);
	}*/

	inline uint8_t toGun(float val) {
		return clampToU8(127.5f + (val * 127.5f));
	}

	inline void writeNormalValue(const Vector3 &vec, Colour24 &output) {
		output.red = toGun(vec.y);
		output.green = toGun(vec.x);
		output.blue = clampToU8(vec.z * 255.0f);
	}

	inline Vector3 createSurfaceNormal(const Vector3 &from, const Vector3 &to) {
		return (Vector3::unitZ() + from - to).normalized();
	}
}

/*void motu::GenerateNormalAndOcclusianMap(const Mesh &high, const Mesh &low, Image &output) {
	int width = output.width(), height = output.height();
	Grid<Mesh::VertexAndNormal> van(width, height);
	high.rasterize(van);
	Grid<Vector3> onto(width, height);
	low.rasterizeNormalsOnly(onto);
	for (int i = 0, j = output.width() * output.height(); i != j; ++i) {
		output.data()[i] = createNormalValue((Vector3::unitZ() + van.data()[i].normal - onto.data()[i]).normalized()) | (static_cast<uint32_t>(computeOcclusian(van, i)) << 24);
	}
}*/

void motu::GenerateNormalMap(const Mesh &high, const Mesh &low, Image &output) {
	int width = output.width(), height = output.height();
	Grid<Mesh::VertexAndNormal> van(width, height);
	high.rasterize(van);
	Grid<Vector3> onto(width, height);
	low.rasterizeNormalsOnly(onto);
	for (int i = 0, j = output.width() * output.height(); i != j; ++i) {
		writeNormalValue(createSurfaceNormal(van.data()[i].normal, onto.data()[i]), output.data()[i]);
	}
}