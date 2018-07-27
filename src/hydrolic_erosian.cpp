#include "hydrolic_erosian.h"
#include "mesh.h"
#include "mesh_edge_map.h"
#include "rivers.h"
#include "sea_erosian.h"
#include "noise_layer.h"

#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <stack>

using namespace motu;

#ifndef MOTU_FLATNESS_POW
#define MOTU_FLATNESS_POW 12
#endif

namespace {

	template<class ErosianDest>
	void trackErosian(Mesh &mesh, const MeshEdgeMap &edgeMap, int offset, float carrying, float carryCapacity, ErosianDest dest) {
		Vector3 *next = &mesh.vertices[offset];
		float speed = 0.0f, lowest = next->z;
		int lastDown = offset;
		while (true) {
			int down = -1;
			std::pair<const int *, const int *> neighbours(edgeMap.vertex(offset));
			while (neighbours.first != neighbours.second){
				const Vector3 &vert = mesh.vertices[*neighbours.first];
				if (vert.z < lowest) {
					down = *neighbours.first;
					lowest = vert.z;
				}
				++neighbours.first;
			}
			if (lowest < 0.0f) {
				dest(mesh, lastDown, carrying);
				return;
			}
			if (down == -1) {
				dest(mesh, lastDown, carrying);
				return;
			}
			Vector3 *lowestVert = &mesh.vertices[down];
			Vector3 direction = *next - *lowestVert;
			float distance = direction.magnitude();
			float acceleration = direction.z / distance;
			acceleration *= acceleration * acceleration * distance;
			speed += acceleration;
			speed *= 0.5f;
			dest(mesh, lastDown, carrying - speed);
			lastDown = down;
			next = lowestVert;
			carrying = speed;
		}
	}

	struct TreePlacer {
		NoiseLayer broad, fine;
		Decoration &decoration;

		TreePlacer(const Vector2 &a, const Vector2 &b, Decoration &decoration) : broad(a, 24.0f), fine(b, 96.0f), decoration(decoration){}

		float operator()(const Vector2 &offset, float richness) {
			richness *= broad.get(offset);
			if (richness > 0.2f) {
				decoration.trees.push_back(offset);
				return richness;
			}
			richness *= fine.get(offset);
			if (richness > 0.1f) {
				decoration.bushes.push_back(offset);
			}
			return richness;
		}

		/*float getForestStrength(const Vector2 &offset, float richness) {
			return (broad.get(offset) + (fine.get(offset) * 0.5f)) * richness;
		}*/
	};

	template <class Tracker>
	Mesh &erode(Mesh &mesh, Tracker tracker) {
		const MeshEdgeMap &edges = mesh.edgeMap();
		std::vector<std::pair<int, float>> sort(mesh.vertices.size());
		for (size_t i = 0; i != sort.size(); ++i) {
			sort[i] = std::make_pair(static_cast<int>(i), mesh.vertices[i].z);
		}
		std::sort(sort.begin(), sort.end(), [](std::pair<int, float> &a, std::pair<int, float> &b) {
			return a.second > b.second;
		});
		for (const auto &i : sort) {
			if (i.second < 0.0f) {
				return mesh;
			}
			trackErosian(mesh, edges, i.first, 0.0f, 0.0f, tracker);
		}
		return mesh;
	}
}

Mesh &motu::applyHydrolicErosian(std::default_random_engine &rd, Decoration &decoration) {
	decoration.mesh.ensureNormals();
	decoration.soilRichness.resize(decoration.mesh.vertices.size(), 0.0f);
	erode(decoration.mesh, [&decoration](Mesh &mesh, int offset, float shift) {
		mesh.vertices[offset].z += shift;
		decoration.soilRichness[offset] += shift;
	});
	float max = std::numeric_limits<float>::min();
	float min = std::numeric_limits<float>::max();
	for (float f : decoration.soilRichness) {
		if (f < min) {
			min = f;
		}
		if (f > max) {
			max = f;
		}
	}
	float mul = 1.0f / (max - min);
	for (float &f : decoration.soilRichness) {
		f = (f - min) * mul;
	}
	Vector3 up = Vector3::unitZ();
	std::uniform_real_distribution<float> quarter(0.0f, 0.25f);
	std::uniform_real_distribution<float> one(0.0f, 1.0f);
	std::uniform_real_distribution<float> half(0.0f, 0.5f);
	std::uniform_real_distribution<float> extraTree(0.0f, 0.35f);
	std::uniform_real_distribution<float> middleA(0.125f, 0.4375f), middleB(0.5625f, 0.875f), across(-0.5f, 0.5f);
	TreePlacer treePlacer(Vector2(one(rd), one(rd)), Vector2(one(rd), one(rd)), decoration);
	decoration.forest.resize(decoration.mesh.vertices.size());
	for (size_t i = 0; i != decoration.soilRichness.size(); ++i) {
		if (decoration.mesh.vertices[i].z <= 0.0f) {
			continue;
		}
		float soil = decoration.soilRichness[i] * powf(decoration.mesh.normals[i].dot(up), MOTU_FLATNESS_POW);
		const Vector2 &pos = decoration.mesh.vertices[i].asVector2();
		if (half(rd) > soil) {
			decoration.smallRocks.push_back(pos);
		}
		else{
			decoration.forest[i] = treePlacer(pos, soil);
		}
	}
	static float third = 1.0f / 3.0f;
	for (size_t i = 0; i < decoration.mesh.triangles.size(); i += 3) {
		float richness = 0.0f;
		int forestCount = 0;
		int a = decoration.mesh.triangles[i], b = decoration.mesh.triangles[i + 1], c = decoration.mesh.triangles[i + 2];
		if (decoration.mesh.vertices[a].z <= 0.0f || decoration.mesh.vertices[b].z <= 0.0f || decoration.mesh.vertices[c].z <= 0.0f) {
			continue;
		}
		richness += decoration.soilRichness[a] * powf(decoration.mesh.normals[a].dot(up), MOTU_FLATNESS_POW);
		richness += decoration.soilRichness[b] * powf(decoration.mesh.normals[b].dot(up), MOTU_FLATNESS_POW);
		richness += decoration.soilRichness[c] * powf(decoration.mesh.normals[c].dot(up), MOTU_FLATNESS_POW);
		richness *= third;
		Triangle tri(decoration.mesh.vertices[a].asVector2(), decoration.mesh.vertices[b].asVector2(), decoration.mesh.vertices[c].asVector2());
		Vector2 centre = tri.findCentroid();
		if (float adj = treePlacer(centre, richness) > 0.1f) {
			for (int i = 0; i != 3; ++i) {
				if (extraTree(rd) < adj) {
					++forestCount;
					Vector2 dir = tri.vertices[i] - centre;
					Vector2 perp = dir.perp();
					treePlacer(centre + dir * middleA(rd) + perp * across(rd), richness);
					treePlacer(centre + dir * middleB(rd) + perp * across(rd), richness);
				}
			}
		}
		float f = static_cast<float>(forestCount);
		decoration.forest[a] += f;
		decoration.forest[b] += f;
		decoration.forest[c] += f;
	}
	max = 0.0f;
	for (float f : decoration.forest) {
		if (f > max) {
			max = f;
		}
	}
	max = 1.0f / max;
	for (float &f : decoration.forest) {
		if (f > max) {
			f *= max;
		}
	}
	return decoration.mesh;
}

Mesh &motu::applyHydrolicErosian(Mesh &mesh) {
	return erode(mesh, [](Mesh &mesh, int offset, float shift) {
		mesh.vertices[offset].z += shift;
	});
}