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

namespace {

	template<class ErosianDest>
	void trackErosian(Mesh &mesh, const MeshEdgeMap &edgeMap, int offset, float carrying, float carryCapacity, ErosianDest dest) {
		Vector3 *next = &mesh.vertices[offset];
		/*if (next->z < 0.0f) {
			return;
		}*/
		//float speed = carrying / carryCapacity, lowest = next->z;
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
			/*speed = speed * 7.0f + acceleration;//next->z - lowest;
			speed *= 0.125f;*/
			//float capacity = speed/* * speed * carryCapacity*/;
			dest(mesh, lastDown, carrying - speed);
			lastDown = down;
			next = lowestVert;//&mesh.vertices[down];
			//dest(mesh, down, /*capacity*/speed - carrying);
			carrying = speed;//capacity;
		}
	}

	struct TreePlacer {
		NoiseLayer broad, fine;
		Decoration &decoration;

		TreePlacer(const Vector2 &a, const Vector2 &b, Decoration &decoration) : broad(a, 32.0f), fine(b, 128.0f), decoration(decoration){}

		bool operator()(const Vector2 &offset, float richness) {
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
	};

	template <class Tracker>
	Mesh &erode(Mesh &mesh, int drops, Tracker tracker) {
		/*float carryCapacity = 1000.0f / mesh.vertices.size();*/
		const MeshEdgeMap &edges = mesh.edgeMap();
		/*while (drops-- > 0) {
			for (int i = 0; i != mesh.vertices.size(); ++i) {
				if (mesh.vertices[i].z > 0.0f) {
					trackErosian(mesh, edges, i, 0.0f, carryCapacity, tracker);
				}
			}
		}
		return mesh;*/
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

Mesh &motu::applyHydrolicErosian(std::default_random_engine &rd, Mesh &mesh, int drops, std::vector<float> &soilRichness, Decoration &decoration) {
	soilRichness.reserve(mesh.vertices.size());
	for (size_t i = 0; i != mesh.vertices.size(); ++i) {
		soilRichness.push_back(0.0f);
	}
	erode(mesh, drops, [&soilRichness](Mesh &mesh, int offset, float shift) {
		mesh.vertices[offset].z += shift;
		soilRichness[offset] += shift;
	});
	float max = std::numeric_limits<float>::min();
	float min = std::numeric_limits<float>::max();
	for (float f : soilRichness) {
		if (f < min) {
			min = f;
		}
		if (f > max) {
			max = f;
		}
	}
	float mul = 1.0f / (max - min);
	for (float &f : soilRichness) {
		f = (f - min) * mul;
	}
	mesh.ensureNormals();
	Vector3 up = Vector3::unitZ();
	std::uniform_real_distribution<float> quarter(0.0f, 0.25f);
	std::uniform_real_distribution<float> one(0.0f, 1.0f);
	std::uniform_real_distribution<float> half(0.0f, 0.4f);
	std::uniform_real_distribution<float> middleA(0.125f, 0.4375f), middleB(0.5625f, 0.875f), across(-0.5f, 0.5f);
	TreePlacer treePlacer(Vector2(one(rd), one(rd)), Vector2(one(rd), one(rd)), decoration);
	for (size_t i = 0; i != soilRichness.size(); ++i) {
		if (mesh.vertices[i].z <= 0.0f) {
			continue;
		}
		float flatness = mesh.normals[i].dot(up);
		flatness *= flatness;
		float soil = soilRichness[i] * flatness * flatness;
		const Vector2 &pos = mesh.vertices[i].asVector2();
		if (half(rd) > soil) {
			decoration.smallRocks.push_back(pos);
		}
		else{
			treePlacer(pos, soil);
		}
	}
	static float third = 1.0f / 3.0f;
	for (size_t i = 0; i < mesh.triangles.size(); i += 3) {
		float richness = 0.0f;
		int a = mesh.triangles[i], b = mesh.triangles[i + 1], c = mesh.triangles[i + 2];
		if (mesh.vertices[a].z <= 0.0f || mesh.vertices[b].z <= 0.0f || mesh.vertices[c].z <= 0.0f) {
			continue;
		}
		float flatness = mesh.normals[a].dot(up);
		flatness *= flatness;
		richness += soilRichness[a] * flatness * flatness;
		flatness = mesh.normals[b].dot(up);
		flatness *= flatness;
		richness += soilRichness[b] * flatness * flatness;
		flatness = mesh.normals[c].dot(up);
		flatness *= flatness;
		richness += soilRichness[c] * flatness * flatness;
		richness *= third;
		Triangle tri(mesh.vertices[a].asVector2(), mesh.vertices[b].asVector2(), mesh.vertices[c].asVector2());
		Vector2 centre = tri.findCentroid();
		if (float adj = treePlacer(centre, richness) > 0.1f) {
			for (int i = 0; i != 3; ++i) {
				if (half(rd) < adj) {
					Vector2 dir = tri.vertices[i] - centre;
					Vector2 perp = dir.perp();
					treePlacer(centre + dir * middleA(rd) + perp * across(rd), richness);
					treePlacer(centre + dir * middleB(rd) + perp * across(rd), richness);
				}
			}
		}
	}
	return mesh;
}

Mesh &motu::applyHydrolicErosian(Mesh &mesh, int iterations) {
	return erode(mesh, iterations, [](Mesh &mesh, int offset, float shift) {
		mesh.vertices[offset].z += shift;
	});
}