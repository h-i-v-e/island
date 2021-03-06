#include <stack>
#include <limits>
#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <memory>

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
#include "raster.h"
#include "z_axis_collider.h"
#include "river_mesh_builder.h"

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
			lods[j].calculateNormals(lods[j]);
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

	void erode(std::default_random_engine &rd, Mesh &mesh, const MeshEdgeMap &mem, const Decoration &decoration, float maxHeight, const Island::Options &options) {
		MeshEdgeMap mep(mesh);
		applyHydrolicErosian(mesh, mep, decoration);
		Rivers rivers(mesh, mem, maxHeight, 0.0f);
		rivers.jiggle(mesh);
		rivers.smooth(mesh, mem);
		rivers.carveInto(mesh, mem, true);
	}
	
	template <class Apply>
	void erode(std::default_random_engine &rd, Mesh &nw, const MeshEdgeMap &mem, float maxHeight, const Island::Options &options, float sd, Apply apply) {
		addNoise(nw, rd, 32.0f, options.noiseMultiplier);
		addNoise(nw, rd, 64.0f, options.noiseMultiplier * 0.5f);
		apply(nw);
		addNoise(nw, rd, 64.0f, options.noiseMultiplier * 0.25f);
		addNoise(nw, rd, 128.0f, options.noiseMultiplier * 0.125f);
		Rivers rivers(nw, mem, maxHeight, sd);
		rivers.jiggle(nw);
		rivers.carveInto(nw, mem, true);
	}

	void createRiverMeshFromLod0(Decoration &decoration, const Rivers &rivers, Mesh &mesh, const MeshEdgeMap &mem, const MeshTriangleMap &mtm, MeshWithUV &output) {
		mesh.calculateNormals(mtm);
		motu::createRiverMesh(8192.0f, decoration, rivers.riverList(), mesh, output, mem, mtm);
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

	struct SeaFinder {
		const Mesh &mesh;
		const MeshEdgeMap &mep;
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

		SeaFinder(const Mesh &mesh, const MeshEdgeMap &mep) : mesh(mesh), mep(mep), sea(mesh.vertices.size(), false), pool(32){
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
				for (auto i = mep.vertex(fringe); i.first != i.second; ++i.first) {
					if (!sea[*i.first]) {
						sea[*i.first] = true;
						stack.push_back(*i.first);
					}
				}
			}
		}

		void findSea(int vert, std::vector<int> &verts) {
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
				for (auto i = mep.vertex(node->vertex); i.first != i.second; ++i.first) {
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

	void forceMinimumSeaDepth(Mesh &mesh, const MeshEdgeMap &mem, float minDepth) {
		for (int i = 0, j = static_cast<int>(mesh.vertices.size()); i != j; ++i) {
			float z = mesh.vertices[i].z;
			if (z < FLT_EPSILON && z > minDepth) {
				bool sink = true;
				for (auto n = mem.vertex(i); n.first != n.second; ++n.first) {
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

void Island::generateNormalMap(int lod, Image24 &nao) const {
	GenerateNormalMap(lods[0], lods[lod], nao);
}

void Island::generateTopology(std::default_random_engine &rd, const Options &options) {
	createInitialTerrain(rd, lods[2]);
	maxHeight = generateSeas(lods[2], rd, maxZ, options);
	lods[2].tesselate();
	{
		MeshEdgeMap mem(lods[2]);
		erode(rd, lods[2], mem, mDecoration, maxZ, options);
		lods[2].smooth(mem);
		improveCliffs(lods[2], mem, mDecoration);
	}
	lods[1] = lods[2];
	lods[1].tesselate();
	{
		MeshEdgeMap mem(lods[1]);
		lods[1].smooth(mem, mDecoration);
		erode(rd, lods[1], mem, maxZ, options, 1.0f, [this, &mem](Mesh &nw) {
			applyHydrolicErosian(nw, mem, mDecoration);
		});
		improveCliffs(lods[1], mem, mDecoration);
	}
	lods[1].tesselate();
	{
		MeshEdgeMap mem(lods[1]);
		lods[1].smooth(mem, mDecoration);
		erode(rd, lods[1], mem, maxZ, options, 2.0f, [this, &mem](Mesh &nw) {
			applyHydrolicErosian(nw, mem, mDecoration);
		});
		improveCliffs(lods[1], mem, mDecoration);
	}
	lods[0] = lods[1];
	lods[0].tesselate();
	{
		MeshEdgeMap mem(lods[0]);
		lods[0].smooth(mem, mDecoration);
		applyHydrolicErosian(rd, lods[0], mem, mDecoration);
		improveCliffs(lods[0], mem, mDecoration);
	}
	lods[0].tesselateIfPositiveZ();
	{
		MeshEdgeMap mem(lods[0]);
		mDecoration.spreadSoilRichness(lods[0], mem);
		erode(rd, lods[0], mem, maxZ, options, 4.0f, [this, &mem](Mesh &nw) {
			applyHydrolicErosian(nw, mem, mDecoration, true);
		});
		lods[0].smooth(mem, mDecoration);
		improveCliffs(lods[0], mem, mDecoration);
	}
	lods[0].tesselateIfPositiveZ();
	{
		MeshEdgeMap mem(lods[0]);
		mDecoration.spreadSoilRichness(lods[0], mem);
		lods[0].smooth(mem, mDecoration);
		applyHydrolicErosian(lods[0], mem, mDecoration, true);
		improveCliffs(lods[0], mem, mDecoration);
		applyHydrolicErosian(lods[0], mem, mDecoration, true);
		forceMinimumSeaDepth(lods[0], mem, -0.0005f);
		lods[0].smoothIfNegativeZ(mem);
		Rivers rivers(lods[0], mem, maxHeight, 64.0f);
		createRiverMeshFromLod0(mDecoration, rivers, lods[0], mem, lods[0], mRiverMesh);
		mDecoration.computePositions(lods[0], mem);
	}
	correctLods(lods);
}

std::ostream& motu::operator<<(std::ostream &out, const Island &island) {
	out.write(reinterpret_cast<const char *>(&island.maxZ), sizeof(island.maxZ));
	out.write(reinterpret_cast<const char *>(&island.maxHeight), sizeof(island.maxHeight));
	for (size_t i = 0; i != 3; ++i) {
		out << island.lods[i];
	}
	return out << island.mDecoration;
}

std::istream& motu::operator>>(std::istream &in, Island &island) {
	in.read(reinterpret_cast<char *>(&island.maxZ), sizeof(island.maxZ));
	in.read(reinterpret_cast<char *>(&island.maxHeight), sizeof(island.maxHeight));
	for (size_t i = 0; i != 3; ++i) {
		in >> island.lods[i];
	}
	in >> island.mDecoration;
	island.mDecoration.computePositions(island.lods[0], island.lods[0]);
	return in;
}