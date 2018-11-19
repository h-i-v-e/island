#include <stack>
#include <limits>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <queue>

#include "island.h"
#include "noise_layer.h"
#include "bounding_box.h"
#include "dalauney.h"
#include "hydrolic_erosian.h"
#include "mesh_edge_map.h"
#include "rivers.h"
#include "mesh_triangle_map.h"
#include "matrix4.h"
#include "colour.h"
#include "raster.h"
#include "sea_erosian.h"
#include "lake.h"
#include "face_erosian.h"
#include "object_pool.h"
#include "bounding_rectangle.h"
#include "mesh_utils.h"
#include "texture_utils.h"

using namespace motu;

namespace {

	void MapSea(const Mesh &mesh, const MeshEdgeMap &edges, const std::vector<bool> &data, std::unordered_set<int> &sea) {
		std::stack<int> unvisited;
		Mesh::PerimeterSet perimeter;
		mesh.getPerimeterSet(perimeter);
		for (int i : perimeter) {
			unvisited.push(i);
		}
		while (!unvisited.empty()) {
			int vert = unvisited.top();
			unvisited.pop();
			auto neighbours = edges.vertex(vert);
			while (neighbours.first != neighbours.second) {
				if (data[*neighbours.first]){
					auto i = sea.find(*neighbours.first);
					if (i == sea.end()) {
						sea.emplace_hint(i, *neighbours.first);
						unvisited.push(*neighbours.first);
					}
				}
				++neighbours.first;
			}
		}
	}

	void RemoveLakes(std::vector<bool> &data, const Mesh &mesh, const MeshEdgeMap &edges) {
		std::unordered_set<int> sea;
		MapSea(mesh, edges, data, sea);
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			if (data[i] && sea.find(i) == sea.end()) {
				data[i] = false;
			}
		}
	}

	struct DistanceToSea {
		const std::vector<bool> &sea;

		DistanceToSea(const std::vector<bool> &sea) : sea(sea) {}

		bool operator()(int index) const {
			return sea[index];
		}
	};

	struct DistanceToLand {
		const std::vector<bool> &sea;

		DistanceToLand(const std::vector<bool> &sea) : sea(sea) {}

		bool operator()(int index) const {
			return !sea[index];
		}
	};

	struct SeaDistanceInc {
		const float multiplier;
		float current;

		SeaDistanceInc(const Island::Options &options) : multiplier(options.slopeMultiplier), current(1.0f) {}

		float operator()(float last) {
			float out = last + current;
			current *= multiplier;
			return out;
		}
	};

	float computeDistanceToSea(const Mesh &mesh, const MeshEdgeMap &edges, const std::vector<bool> &sea, std::vector<float> &distanceVec, const Island::Options &options) {
		return computeDistance(DistanceToSea(sea), mesh, edges, distanceVec, SeaDistanceInc(options));
	}

	float computeDistanceToLand(const Mesh &mesh, const MeshEdgeMap &edges, const std::vector<bool> &sea, std::vector<float> &distanceVec) {
		return computeDistance(DistanceToLand(sea), mesh, edges, distanceVec);
	}

	void setZValues(std::vector<Vector3> &vertices, const std::vector<float> &distanceToSea, const std::vector<float> &distanceToLand, float maxDistance, float maxZ) {
		float mul = maxZ / maxDistance;
		for (int i = 0; i != vertices.size(); ++i) {
			vertices[i].z = (distanceToSea[i] - distanceToLand[i]) * mul;
		}
	}

	void addNoise(Mesh &mesh, std::default_random_engine &rnd, float scale, float maxZShift) {
		std::uniform_real_distribution<float> dis(0.0f, 1.0f);
		NoiseLayer noise(Vector2(dis(rnd), dis(rnd)), scale);
		for (Vector3 &vert : mesh.vertices) {
			if (vert.z > 0.0f) {
				vert.z += noise.getPlusMinus(vert.asVector2()) * maxZShift;
			}
		}
	}

	float generateSeas(Mesh &terrain, std::default_random_engine &rnd, float &maxZ, const Island::Options &options) {
		std::uniform_real_distribution<float> dis(0.0f, 1.0f);
		maxZ = std::normal_distribution<float>(maxZ, maxZ * 0.2f)(rnd);
		std::vector<bool> seas(terrain.vertices.size(), false);
		std::vector<float> distanceToSea(terrain.vertices.size(), std::numeric_limits<float>::max());
		std::vector<float> distanceToLand(terrain.vertices.size(), std::numeric_limits<float>::max());
		MeshEdgeMap edges(terrain);
		NoiseLayer layers[4];
		float strength = 2.0f;
		for (int i = 0; i != 4; ++i) {
			layers[i] = NoiseLayer(Vector2(dis(rnd), dis(rnd)), strength);
			strength *= 4.0f;
		}
		int seaCount = 0;
		static Vector3 centre(0.5f, 0.5f, 0.0f);
		for (int i = 0; i != terrain.vertices.size(); ++i) {
			float noise = 0.0f;
			float dilute = 1.0f;
			for (int j = 0; j != 4; ++j) {
				noise += layers[j].get(terrain.vertices[i].asVector2()) / dilute;
				dilute *= 2.0f;
			}
			noise *= 1.0f - ((terrain.vertices[i] - centre).magnitude() * 1.5f);
			if (seas[i] = noise < options.waterRatio) {
				++seaCount;
			}
		}
		RemoveLakes(seas, terrain, edges);
		float maxHeight = computeDistanceToSea(terrain, edges, seas, distanceToSea, options);
		computeDistanceToLand(terrain, edges, seas, distanceToLand);

		setZValues(terrain.vertices, distanceToSea, distanceToLand, maxHeight, maxZ);
		return maxHeight;
	}

	void correctLods(Mesh *lods) {
		const Mesh::Vertices &verts0 = lods[0].vertices;
		Mesh::Vertices &verts1 = lods[1].vertices, &verts2 = lods[2].vertices;
		size_t i = 0;
		for (size_t j = verts2.size(); i < j; ++i) {
			verts1[i] = verts2[i] = verts0[i];
		}
		for (size_t j = verts1.size(); i < j; ++i) {
			verts1[i] = verts0[i];
		}
		for (size_t j = 0; j != 3; ++j) {
			lods[j].calculateNormals();
		}
	}

	Vector3 vector2ToVector3(const Vector2 &vec) {
		return Vector3(vec.x, vec.y, 0.0f);
	}

	void createSeedPoints(std::default_random_engine &rd, int number, std::vector<Vector3> &points) {
		points.reserve(number);
		std::uniform_real_distribution<float> dis(0.0f, 1.0f);
		for (int i = 0, j = number - 8; i < j; ++i) {
			points.emplace_back(dis(rd), dis(rd), 0.0f);
		}
		points.emplace_back(0.0f, 0.0f, 0.0f);
		points.emplace_back(0.0f, 0.5f, 0.0f);
		points.emplace_back(0.0f, 1.0f, 0.0f);
		points.emplace_back(0.5f, 1.0f, 0.0f);
		points.emplace_back(1.0f, 1.0f, 0.0f);
		points.emplace_back(1.0f, 0.5f, 0.0f);
		points.emplace_back(1.0f, 0.0f, 0.0f);
		points.emplace_back(0.5f, 0.0f, 0.0f);
	}

	void createInitialTerrain(std::default_random_engine &rd, Mesh &mesh) {
		std::vector<Vector3> points;
		createSeedPoints(rd, 1024, points);
		createDalauneyMesh(points, mesh);
	}

	Rivers *erode(std::default_random_engine &rd, Mesh &mesh, const RockSet &rock, float maxHeight, const Island::Options &options) {
		MeshEdgeMap mep(mesh);
		applyHydrolicErosian(mesh, rock);
		Rivers *rivers = new Rivers(mesh, maxHeight, 0.0f);
		rivers->jiggle(mesh);
		rivers->smooth(mesh);
		rivers->carveInto(mesh, true);
		//smoothPeeks(mesh);
		return rivers;
	}
	
	template <class Apply>
	void erode(std::default_random_engine &rd, Rivers **river, Mesh &nw, float maxHeight, const Island::Options &options, float sd, Apply apply) {
		addNoise(nw, rd, 32.0f, options.noiseMultiplier);
		addNoise(nw, rd, 64.0f, options.noiseMultiplier * 0.5f);
		apply(nw);
		addNoise(nw, rd, 64.0f, options.noiseMultiplier * 0.25f);
		addNoise(nw, rd, 128.0f, options.noiseMultiplier * 0.125f);
		Rivers *rivers = new Rivers(nw, maxHeight, sd);
		rivers->jiggle(nw);
		//rivers->smooth(nw);
		rivers->carveInto(nw, true);
		//smoothPeeks(nw);
		delete *river;
		*river = rivers;
	}

	void finalRiverPass(Rivers **rivers, Mesh &mesh, float maxHeight, const Island::Options &options) {
		delete *rivers;
		MeshEdgeMap mep(mesh);
		*rivers = new Rivers(mesh, maxHeight, 64.0f);
		(*rivers)->jiggle(mesh);
		(*rivers)->smooth(mesh);
		//smoothPeeks(mesh);
		//(*rivers)->carveInto(mesh, options.maxRiverGradient);
	}

	void createRiverMeshes(const Rivers &rivers, const Mesh &mesh, Island::RiverMeshes &meshes) {
		meshes.reserve(rivers.riverList().size());
		Mesh buffer;
		BoundingBox bounds(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
		for (const auto &i : rivers.riverList()) {
			buffer.clear();
			rivers.getMesh(*i, mesh, buffer);
			if (buffer.vertices.empty()) {
				continue;
			}
			meshes.push_back(std::make_shared<Mesh>());
			buffer.slice(bounds, *meshes.back());
			if (meshes.back()->vertices.empty()) {
				meshes.pop_back();
			}
		}
	}

	struct GetShifted {
		std::unordered_map<Vector3, Vector3, Hasher<Vector3>> shiftMap;

		const Vector3 &operator()(const Vector3 &old, const Vector3 &nw) {
			auto i = shiftMap.find(old);
			if (i == shiftMap.end()) {
				shiftMap.emplace_hint(i, old, nw);
				return nw;
			}
			return i->second;
		}
	};

	void smoothRiverLists(Island::RiverVertexLists &lists) {
		GetShifted getShifted;
		for (auto &riv : lists) {
			//riv->reserve(riv->size() << 1);
			std::vector<Island::RiverVertex> buffer;
			buffer.reserve(riv->size() << 1);
			buffer.push_back(riv->front());
			for (size_t i = 1; i < riv->size(); ++i) {
				buffer.emplace_back(((*riv)[i - 1].vertex + (*riv)[i].vertex) * 0.5f, (*riv)[i - 1].flow);
				buffer.push_back((*riv)[i]);
			}
			riv->clear();
			static float third = 1.0f / 3.0f;
			riv->emplace_back(getShifted(buffer.front().vertex, buffer.front().vertex), buffer.front().flow);
			for (size_t i = 1, j = buffer.size() - 1; i < j; ++i) {
				riv->emplace_back(
					getShifted(
						buffer[i].vertex,
						(buffer[i - 1].vertex + buffer[i].vertex + buffer[i + 1].vertex) * third
					),
					buffer[i].flow
				);
			}
			riv->emplace_back(getShifted(buffer.back().vertex, buffer.back().vertex), buffer.back().flow);
		}
	}

	struct SeaFinder {
		const Mesh &mesh;
		std::vector<bool> sea;
		std::unordered_set<int> visited;

		struct PathNode {
			const PathNode *last;
			int vertex;
			float cost;
		};

		struct Comp {
			bool operator()(const PathNode *a, const PathNode *b) const {
				return a->cost < b->cost;
			}
		};

		ObjectPool<PathNode> pool;
		std::priority_queue<PathNode*, std::vector<PathNode*>, Comp> queue;

		SeaFinder(const Mesh &mesh) : mesh(mesh), sea(mesh.vertices.size(), false), pool(32){
			const MeshEdgeMap &mem = mesh.edgeMap();
			std::vector<int> stack;
			stack.reserve(sea.size());
			{
				Mesh::PerimeterSet ps;
				mesh.getPerimeterSet(ps);
				for (int i : ps) {
					sea[i] = true;
					stack.push_back(i);
				}
			}
			while (!stack.empty()) {
				int fringe = stack.back();
				stack.pop_back();
				for (auto i = mem.vertex(fringe); i.first != i.second; ++i.first) {
					if (!sea[*i.first]) {
						sea[*i.first] = true;
						stack.push_back(*i.first);
					}
				}
			}
		}

		void findSea(int vert, std::vector<int> &verts) {
			const MeshEdgeMap &mem = mesh.edgeMap();
			PathNode *node = pool.allocate();
			node->last = nullptr;
			node->vertex = vert;
			node->cost = mesh.vertices[vert].z;
			queue.push(node);
			visited.insert(vert);
			while (!queue.empty()) {
				node = queue.top();
				if (sea[node->vertex]) {
					break;
				}
				queue.pop();
				for (auto i = mem.vertex(node->vertex); i.first != i.second; ++i.first) {
					if (visited.find(*i.first) != visited.end()) {
						continue;
					}
					visited.insert(*i.first);
					PathNode *next = pool.allocate();
					next->vertex = *i.first;
					next->last = node;
					next->cost = mesh.vertices[next->vertex].z + node->cost;
					queue.push(next);
				}
			}
			for (const PathNode *i = node; i; i = i->last) {
				verts.push_back(i->vertex);
			}
			std::reverse(verts.begin(), verts.end());
		}
	};

	void createRiverLists(const Mesh &mesh, const Rivers &rivers, Island::RiverVertexLists &lists) {
		int len = rivers.riverList().size();
		lists.reserve(len);
		std::unordered_set<int> used;
		std::vector<int> toSeaList;
		SeaFinder seaFinder(mesh);
		for (int i = 0; i != len; ++i) {
			used.clear();
			toSeaList.clear();
			auto list = std::make_shared<Island::VertexList>();
			list->reserve(rivers.riverList()[i]->vertices.size() << 1);
			for (const auto &j : rivers.riverList()[i]->vertices) {
				if (used.find(j.index) == used.end()) {
					list->emplace_back(mesh.vertices[j.index], j.flow);
					used.insert(j.index);
				}
			}
			const auto &k = rivers.riverList()[i]->vertices.back();
			seaFinder.findSea(k.index, toSeaList);
			for (size_t j = 1; j < toSeaList.size(); ++j) {
				list->emplace_back(mesh.vertices[j], k.flow);
			}
			lists.push_back(list);
		}
		smoothRiverLists(lists);
	}

	/*void lowerMesh(Mesh &mesh, float amount) {
		for (Vector3 &vert : mesh.vertices) {
			vert.z -= amount;
		}
	}*/

	void forceMinimumSeaDepth(Mesh &mesh, float minDepth) {
		const MeshEdgeMap &mep = mesh.edgeMap();
		for (int i = 0, j = static_cast<int>(mesh.vertices.size()); i != j; ++i) {
			float z = mesh.vertices[i].z;
			if (z < FLT_EPSILON && z > minDepth) {
				bool sink = true;
				for (auto n = mep.vertex(i); n.first != n.second; ++n.first) {
					if (mesh.vertices[*n.first].z > FLT_EPSILON) {
						sink = false;
						break;
					}
				}
				if (sink) {
					mesh.vertices[i].z = minDepth;
				}
			}
		}
	}

}

void Island::generateNormalAndOcclusianMap(Image &nao) const {
	//GenerateNormalAndOcclusianMap(lods[0], lods[2], nao);
}

void Island::generateNormalMap(int lod, Image24 &nao) const {
	GenerateNormalMap(lods[0], lods[lod], nao);
}

void Island::generateTopology(std::default_random_engine &rd, const Options &options) {
	Mesh preSlice[3];
	RockSet cliffs;
	createInitialTerrain(rd, preSlice[2]);
	//for (int i = 0; i != 2; ++i) {
		//preSlice[2].tesselate();
		//preSlice[2].smooth();
	//}
	maxHeight = generateSeas(preSlice[2], rd, maxZ, options);
	preSlice[2].tesselate();
	Rivers *rivers = erode(rd, preSlice[2], cliffs, maxZ, options);
	preSlice[2].smooth();
	improveCliffs(preSlice[2], cliffs);
	preSlice[1] = preSlice[2];
	preSlice[1].tesselate();
	erode(rd, &rivers, preSlice[1], maxZ, options, 1.0f, [&cliffs](Mesh &nw) {
		applyHydrolicErosian(nw, cliffs);
	});
	//cliffs.clear();
	improveCliffs(preSlice[1], cliffs);
	preSlice[1].tesselate();
	erode(rd, &rivers, preSlice[1], maxZ, options, 2.0f, [&cliffs](Mesh &nw) {
		applyHydrolicErosian(nw, cliffs);
	});
	//cliffs.clear();
	improveCliffs(preSlice[1], cliffs);
	preSlice[0] = preSlice[1];
	preSlice[0].tesselate();
	//preSlice[1].tesselate();
	//eatCoastlines(preSlice[0], 16);

	//lowerMesh(preSlice[0], maxZ * 0.005f);
	erode(rd, &rivers, preSlice[0], maxZ, options, 4.0f, [&cliffs](Mesh &nw) {
		applyHydrolicErosian(nw, cliffs, true);
	});
	preSlice[0].smooth(cliffs);
	//cliffs.clear();
	improveCliffs(preSlice[0], cliffs);
	//preSlice[0] = preSlice[1];
	preSlice[0].tesselateIfPositiveZ();
	erode(rd, &rivers, preSlice[0], maxZ, options, 8.0f, [&cliffs](Mesh &nw) {
		applyHydrolicErosian(nw, cliffs, true);
	});
	improveCliffs(preSlice[0], cliffs);
	preSlice[0].tesselateIfPositiveZ();
	preSlice[0].smooth(cliffs);
	applyHydrolicErosian(preSlice[0], cliffs, true);
	improveCliffs(preSlice[0], cliffs);
	/*preSlice[0].tesselateIfPositiveZ();
	preSlice[0].smooth(cliffs);
	improveCliffs(preSlice[0], cliffs);*/
	//eatCoastlines(preSlice[0], 1);
	mDecoration = std::make_unique<Decoration>(preSlice[0]);
	applyHydrolicErosian(rd, cliffs, *mDecoration);
	preSlice[0] = mDecoration->mesh;
	finalRiverPass(&rivers, preSlice[0], maxZ, options);
	forceMinimumSeaDepth(preSlice[0], -0.0005f);
	createRiverMeshes(*rivers, preSlice[0], mRiverMeshes);
	createRiverLists(preSlice[0], *rivers, mRiverVertexLists);
	correctLods(preSlice);
	for (int i = 0; i != 3; ++i) {
		preSlice[i].slice(BoundingBox(0.0f, 0.0f, -0.0025f, 1.0f, 1.0f, 1.0f), lods[i]);
	}
	delete rivers;
	//placeCoastalRocks(rd, *mDecoration);
}