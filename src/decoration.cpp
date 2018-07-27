#include "decoration.h"
#include "mesh.h"

using namespace motu;

namespace {
	float getRockChance(const Mesh &mesh, const MeshEdgeMap &mep, int offset) {
		Vector3 up = Vector3::unitZ();
		float total = mesh.normals[offset].dot(up);
		int count = 1;
		auto neighbours = mep.vertex(offset);
		while (neighbours.first != neighbours.second) {
			total += mesh.normals[*neighbours.first++].dot(up);
			++count;
		}
		return (1.0f - (total / static_cast<float>(count)));
	}

	bool isLocalMaximum(const Mesh &mesh, const MeshEdgeMap &mep, int offset) {
		float z = mesh.vertices[offset].z;
		auto neighbours = mep.vertex(offset);
		while (neighbours.first != neighbours.second) {
			if (mesh.vertices[*neighbours.first++].z >= z) {
				return false;
			}
		}
		return true;
	}
}

void Decoration::addRocks(std::default_random_engine &rnd) {
	std::uniform_real<float> full(0.0f, 1.0f);
	std::uniform_real<float> half(0.5f, 1.0f);
	std::uniform_real<float> quarter(0.25f, 0.75f);
	const MeshEdgeMap &mep = mesh.edgeMap();
	std::unordered_set<int> rocked;
	for (int i = 0, j = static_cast<int>(mesh.vertices.size()); i != j; ++i) {
		if (isLocalMaximum(mesh, mep, i)) {
			const Vector2 &pos = mesh.vertices[i].asVector2();
			if (full(rnd) < getRockChance(mesh, mep, i)) {
				bigRocks.push_back(pos);
			}
			else {
				mediumRocks.push_back(pos);
			}
			rocked.insert(i);
		}
	}
	for (int i : rocked) {
		const Vector2 &pos = mesh.vertices[i].asVector2();
		auto neighbours = mep.vertex(i);
		while (neighbours.first != neighbours.second) {
			int offset = *neighbours.first++;
			if (rocked.find(offset) != rocked.end()) {
				continue;
			}
			Vector2 dir = mesh.vertices[offset].asVector2() - pos;
			if (full(rnd) < (1.0f - Vector3::unitZ().dot(mesh.normals[offset]))) {
				mediumRocks.push_back(pos + dir * half(rnd));
			}
			else {
				smallRocks.push_back(pos + dir * quarter(rnd));
			}
		}
	}
}