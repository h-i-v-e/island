#define _USE_MATH_DEFINES

#include <cmath>

#include "tree_billboards.h"

using namespace motu;

namespace {
	struct Rotator {
		float s, c;
		Vector3 normal;

		Rotator() {}

		Rotator(float radians) : s(sinf(radians)), c(cosf(radians)), normal(c, s, 0.0f) {
		}

		Vector3 operator()(const Vector3 &v3) const{
			return Vector3(
				v3.x * c,
				v3.x * s,
				v3.z
			);
		}
	};

	struct Octant {
		Vector3 target;

		Octant(const Vector3 &target) : target(target) {}

		TreeBillboards::Positions &sort(const TreeBillboards::Positions &offsets, TreeBillboards::Positions &buffer, std::vector<int> &ooff) const{
			std::sort(ooff.begin(), ooff.end(), [this, &offsets](int ai, int bi) {
				const TreeBillboards::PositionScale &a = offsets[ai], &b = offsets[bi];
				return (target - a.position).squareMagnitude() > (target - b.position).squareMagnitude();
			});
			for (size_t i = 0, j = offsets.size(); i != j; ++i) {
				buffer[i] = offsets[ooff[i]];
			}
			return buffer;
		}
	};

	Octant octants[8] = {
		Octant(Vector3(0.0f, -1.0f, 0.0f)),
		Octant(Vector3(1.0f, -1.0f, 0.0f)),
		Octant(Vector3(1.0f, 0.0f, 0.0f)),
		Octant(Vector3(1.0f, 1.0f, 0.0f)),
		Octant(Vector3(0.0f, 1.0f, 0.0f)),
		Octant(Vector3(-1.0f, 1.0f, 0.0f)),
		Octant(Vector3(-1.0f, 0.0f, 0.0f)),
		Octant(Vector3(-1.0f, -1.0f, 0.0f))
	};

	struct Rotators {
		Rotators() {
			static float step = static_cast<float>(M_PI * 0.25);
			for (size_t i = 0; i != 8; ++i) {
				rotations[i] = Rotator(static_cast<float>(i) * step);
			}
		}

		Rotator rotations[8];
	} rotators;

	Vector3 vertices[] = {
		Vector3(-0.5f, 0.0f, 0.0f),
		Vector3(-0.5f, 0.0f, 1.0f),
		Vector3(0.5f, 0.0f, 1.0f),
		Vector3(0.5f, 0.0f, 0.0f)
	};

	int triangles[] = {
		2, 1, 0, 3, 2, 0
	};

	void reserve(Mesh &mesh, size_t size){
		size_t len = size << 2;
		mesh.vertices.resize(len);
		mesh.normals.resize(len);
		mesh.triangles.resize(len + (size << 1));
	}

	void addQuad(Mesh &mesh, const TreeBillboards::PositionScale &pos, const Rotator &rot, size_t offsetV, size_t offsetT) {
		for (size_t j = 0, v = offsetV; j != 4; ++j, ++v) {
			mesh.vertices[v] = rot(vertices[j] * pos.scale) + pos.position;
			mesh.normals[v] = rot.normal;
		}
		for (size_t j = 0; j != 6; ++j, ++offsetT) {
			mesh.triangles[offsetT] = triangles[j] + offsetV;
		}
	}

	std::unique_ptr<TreeBillboards::Offsets> makeOffsets(size_t size) {
		auto out = std::make_unique<TreeBillboards::Offsets>(size);
		for (int i = 0, j = static_cast<int>(size); i != j; ++i) {
			(*out)[i] = i;
		}
		return std::move(out);
	}
}

TreeBillboards::TreeBillboards(const Positions &positions) {
	size_t len = positions.size();
	reserve(*this, len << 1);
	for (size_t i = 0, offsetV = 0, offsetT = 0; i != len; ++i, offsetV += 8, offsetT += 12) {
		const PositionScale &pos = positions[i];
		addQuad(*this, pos, rotators.rotations[i & 7], offsetV, offsetT);
		addQuad(*this, pos, rotators.rotations[(i + 2) & 7], offsetV + 4, offsetT + 6);
	}
}

std::vector<TreeBillboards::UPtr> &TreeBillboards::createOctants(const Positions &positions, std::vector<UPtr> &output, OctantOffsets &octantOffsets) {
	output.resize(8);
	octantOffsets.resize(8);
	size_t size = positions.size();
	Positions buffer(size);
	for (size_t i = 0; i != 8; ++i) {
		octantOffsets[i] = makeOffsets(size);
		output[i] = std::make_unique<TreeBillboards>(octants[i].sort(positions, buffer, *octantOffsets[i]));
	}
	return output;
}