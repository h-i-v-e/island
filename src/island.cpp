#include <stack>
#include <limits>
#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "island.h"
#include "noise_layer.h"
#include "bounding_box.h"
//#include "terrain.h"
#include "dalauney.h"
#include "hydrolic_erosian.h"
#include "mesh_edge_map.h"
#include "rivers.h"
#include "mesh_triangle_map.h"
#include "matrix4.h"
#include "colour.h"
#include "raster.h"
#include "sea_erosian.h"

using namespace motu;

namespace {
	short occlusianCursor[30] = {
		-8, -2, -4, 2, -2, 1, -1, 1,
		-1, -1, -1, -4, 1, 2, 1, 1,
		1, -1, 1, -2, 2, 4, 2, -1,
		2, -8, 4, -2, 8, 2
	};

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
		bool operator()(bool sea) const {
			return sea;
		}
	};

	struct DistanceToLand {
		bool operator()(bool sea) const {
			return !sea;
		}
	};

	template <class DistanceTo>
	float ComputeDistanceTo(const Mesh &mesh, const MeshEdgeMap &edges, const std::vector<bool> &sea, std::vector<float> &distanceVec, DistanceTo dt, float addMul) {
		typedef std::stack<int> Stack;

		Stack circles[2];
		Stack *last = circles;
		Stack *next = circles + 1;

		float distance = 0.0f, add = 0.5f;
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			auto neighbours = edges.vertex(i);
			while (neighbours.first != neighbours.second){
				if (dt(sea[*neighbours.first++])) {
					distanceVec[i] = 0.0f;
					last->push(i);
					break;
				}
			}
		}
		do {
			add *= addMul;
			distance += add;
			while (!last->empty()) {
				int i = last->top();
				last->pop();
				auto neighbours = edges.vertex(i);
				while (neighbours.first != neighbours.second) {
					if (distance < distanceVec[*neighbours.first]) {
						distanceVec[*neighbours.first] = distance;
						next->push(*neighbours.first);
					}
					++neighbours.first;
				}
			}
			Stack *swap = last;
			last = next;
			next = swap;
		} while (!last->empty());
		return distance;
	}

	void setZValues(std::vector<Vector3> &vertices, const std::vector<float> &distanceToSea, const std::vector<float> &distanceToLand, float maxDistance, float maxZ) {
		float mul = maxZ / maxDistance;
		for (int i = 0; i != vertices.size(); ++i) {
			vertices[i].z = (distanceToSea[i] - distanceToLand[i]) * mul;
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
		float maxHeight = ComputeDistanceTo(terrain, edges, seas, distanceToSea, DistanceToSea(), options.slopeMultiplier);
		ComputeDistanceTo(terrain, edges, seas, distanceToLand, DistanceToLand(), 0.75f);

		setZValues(terrain.vertices, distanceToSea, distanceToLand, maxHeight, maxZ);
		return maxHeight;
	}

	uint8_t computeOcclusian(const Grid<Mesh::VertexAndNormal> &grid, int x, int y) {
		float total = 0.0f;
		size_t count = 0;
		const Mesh::VertexAndNormal &van = grid(x, y);
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

	void correctZ(const Grid<Mesh::VertexAndNormal> &grid, Mesh &mesh) {
		for (size_t i = 0; i != mesh.vertices.size(); ++i) {
			size_t x = static_cast<size_t>(mesh.vertices[i].x * grid.width());
			size_t y = static_cast<size_t>(mesh.vertices[i].y * grid.height());
			if (x < 0) {
				x = 0;
			}
			else if (x >= grid.width()) {
				x = grid.width() - 1;
			}
			if (y < 0) {
				y = 0;
			}
			else if (y >= grid.height()) {
				y = grid.height() - 1;
			}
			mesh.vertices[i].z = grid(x, y).vertex.z;
		}
		mesh.calculateNormals();
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

	uint32_t createNormalValue(const Vector3 &vec) {
		uint32_t out = clampToU8(127.5f + (vec.x * 127.5f)) << 16;
		out |= clampToU8(127.5f + (vec.y * 127.5f)) << 8;
		return out | clampToU8(127.5f + (vec.z * 127.5f));
	}

	inline uint32_t computeAlbedo(const Mesh::VertexAndNormal &van, const Island::Pallete &pallete) {
		float slope = van.normal.dot(Vector3::unitZ());
		slope *= slope;
		if ((slope < 0.7f && van.vertex.z < 0.0001) || van.vertex.z <= 0.0f) {
			return toColour32(pallete.sand);
		}
		if (van.vertex.z > 0.05f) {
			return toColour32(pallete.mountain);
		}
		else {
			float rock = van.vertex.z * 20.0f;
			Vector3 base = pallete.grass * (1.0f - rock) + pallete.mountain * rock;
			return toColour32(base * slope + (pallete.cliff * (1.0f - slope)));
		}
	}

	void assignNormalAndOcclusianMap(Island::NormalAndOcclusianMap &map, const Mesh &lod2, const Grid<Vector3> &normals, const Grid<uint8_t> &occlusian) {
		Grid<Vector3> low(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
		lod2.rasterizeNormalsOnly(low);
		static int size = MOTU_TEXTURE_MAP_SIZE * MOTU_TEXTURE_MAP_SIZE;
		for (int i = 0; i != size; ++i) {
			map.data()[i] = createNormalValue((Vector3::unitZ() + normals.data()[i] - low.data()[i]).normalized()) | (static_cast<uint32_t>(occlusian.data()[i]) << 24);
		}
	}

	void correctLodsAndGenerateMaps(Mesh *lods, Island::NormalAndOcclusianMap &nao, Island::AlbedoMap &albedo, const Island::Pallete &pallete, const Rivers &rivers) {
		Grid<Mesh::VertexAndNormal> grid(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
		Raster raster(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
		lods[0].calculateNormals();
		lods[0].rasterize(grid);
		correctZ(grid, lods[1]);
		correctZ(grid, lods[2]);
		Grid<Vector3> normals(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
		Grid<uint8_t> occlusian(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
		for (size_t y = 0, i = 0; y != grid.height(); ++y) {
			for (size_t x = 0; x != grid.width(); ++x, ++i) {
				normals.data()[i] = grid.data()[i].normal;
				occlusian.data()[i] = computeOcclusian(grid, x, y);
				raster.data()[i] = computeAlbedo(grid.data()[i], pallete);
			}
		}
		assignNormalAndOcclusianMap(nao, lods[2], normals, occlusian);
		for (const Rivers::River &river : rivers.rivers()) {
			auto i = river.begin();
			Vector2 last = lods[1].vertices[i->first].asVector2();
			for (++i; i != river.end(); ++i) {
				const Vector2 &next = lods[1].vertices[i->first].asVector2();
				raster.draw(Edge(last, next), toColour32(pallete.cliff));
				last = next;
			}
		}
		for (int i = 0; i != raster.length(); ++i) {
			albedo.data()[i] = toColour16(raster.data()[i]);
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
		createSeedPoints(rd, 512, points);
		createDalauneyMesh(points, mesh);
		for (int i = 0; i != 16; ++i) {
			mesh.smooth();
		}
	}

	Rivers *erode(Mesh &mesh, const Island::Options &options) {
		MeshEdgeMap mep(mesh);
		applyHydrolicErosian(mesh, mep, options.erosianPasses);
		applySeaErosian(mep, mesh);
		Rivers *rivers = new Rivers(mesh, mep);
		rivers->carveInto(mesh, mep, 0.0f, options.maxRiverGradient);
		return rivers;
	}

	void erode(Rivers **river, const Mesh &old, Mesh &nw, const Island::Options &options) {
		MeshEdgeMap mep(nw);
		applyHydrolicErosian(nw, mep, options.erosianPasses);
		Rivers *rivers = new Rivers(**river, old, nw, mep);
		rivers->smooth(nw, mep);
		rivers->carveInto(nw, mep, options.riverDepth, options.maxRiverGradient);
		applySeaErosian(mep, nw);
		delete *river;
		*river = rivers;
	}
}

void Island::generateTopology(std::default_random_engine &rd, const Options &options) {
	createInitialTerrain(rd, lods[2]);
	for (int i = 0; i != 2; ++i) {
		lods[2].tesselate();
		lods[2].smooth();
	}
	maxHeight = generateSeas(lods[2], rd, maxZ, options);
	lods[2].tesselate();
	lods[2].smooth();
	Rivers *rivers = erode(lods[2], options);
	lods[1] = lods[2];
	lods[1].tesselate();
	erode(&rivers, lods[2], lods[1], options);
	lods[0] = lods[1];
	lods[0].tesselate().tesselate().smooth();
	applySeaErosian(lods[0]);
	correctLodsAndGenerateMaps(lods, mNormalAndOcclusianMap, albedo, options.pallete, *rivers);
	delete rivers;
}