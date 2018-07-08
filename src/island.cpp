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
#include "lake.h"
#include "face_erosian.h"

using namespace motu;

namespace {
	short occlusianCursor[30] = {
		-8, -2, -4, 2, -2, 1, -1, 1,
		-1, -1, -1, -4, 1, 2, 1, 1,
		1, -1, 1, -2, 2, 4, 2, -1,
		2, -8, 4, -2, 8, 2
	};

	Colour sandChannel(1.0f, 0.0f, 0.0f, 0.0f), rockChannel(0.0f, 1.0f, 0.0f, 0.0f), grassChannel(0.0f, 0.0f, 1.0f, 0.0f), alpineChannel(0.0f, 0.0f, 0.0f, 1.0f);


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
	float ComputeDistanceTo(const Mesh &mesh, const MeshEdgeMap &edges, const std::vector<bool> &sea, std::vector<float> &distanceVec, DistanceTo dt, const Island::Options &options) {
		typedef std::stack<int> Stack;

		Stack circles[2];
		Stack *last = circles;
		Stack *next = circles + 1;

		float distance = options.coastalSlopeMultiplier, add = 0.5f;
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
			add *= options.slopeMultiplier;
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
			/*float z = distanceToSea[i] - distanceToLand[i];
			if (z < 0.0f) {
				z -= 10.0f;
			}
			vertices[i].z = z * mul;*/
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
		float maxHeight = ComputeDistanceTo(terrain, edges, seas, distanceToSea, DistanceToSea(), options);
		ComputeDistanceTo(terrain, edges, seas, distanceToLand, DistanceToLand(), options);

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
		for (int i = 0; i != mesh.vertices.size(); ++i) {
			int x = static_cast<int>(mesh.vertices[i].x * grid.width());
			int y = static_cast<int>(mesh.vertices[i].y * grid.height());
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

	inline uint32_t computeAlbedo(const Mesh::VertexAndNormal &van) {
		float slope = van.normal.dot(Vector3::unitZ());
		slope *= slope;
		if (van.vertex.z <= 0.0f) {
			return sandChannel;
		}
		if (van.vertex.z < 0.001) {
			return sandChannel * slope + (rockChannel * (1.0f - slope));
		}
		if (van.vertex.z > 0.05f) {
			return alpineChannel * slope + (rockChannel * (1.0f - slope));
		}
		else {
			float rock = van.vertex.z * 20.0f;
			Colour base = grassChannel * (1.0f - rock) + alpineChannel * rock;
			return base * slope + (rockChannel * (1.0f - slope));
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

	void correctLods(Mesh *lods) {
		Grid<Mesh::VertexAndNormal> grid(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
		lods[0].calculateNormals();
		lods[0].rasterize(grid);
		correctZ(grid, lods[1]);
		correctZ(grid, lods[2]);
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

	Rivers *erode(std::default_random_engine &rd, Mesh &mesh, float maxHeight, const Island::Options &options) {
		MeshEdgeMap mep(mesh);
		applyHydrolicErosian(mesh, options.erosianPasses);
		Rivers *rivers = new Rivers(mesh, maxHeight, 0.0f);
		rivers->smooth(mesh);
		rivers->carveInto(mesh, options.maxRiverGradient * 32.0f);
		return rivers;
	}
	
	template <class Apply>
	void erode(std::default_random_engine &rd, Rivers **river/*, const Mesh &old*/, Mesh &nw, float maxHeight, const Island::Options &options, float sd, Apply apply) {
		addNoise(nw, rd, 32.0f, options.noiseMultiplier);
		addNoise(nw, rd, 64.0f, options.noiseMultiplier * 0.5f);
		apply(nw, options.erosianPasses);
		addNoise(nw, rd, 64.0f, options.noiseMultiplier * 0.25f);
		addNoise(nw, rd, 128.0f, options.noiseMultiplier * 0.125f);
		Rivers *rivers = new Rivers(nw, maxHeight, sd);
		rivers->smooth(nw);
		rivers->carveInto(nw, options.maxRiverGradient * 8.0f);
		delete *river;
		*river = rivers;
	}

	void finalRiverPass(Rivers **rivers, Mesh &mesh, float maxHeight, const Island::Options &options) {
		delete *rivers;
		MeshEdgeMap mep(mesh);
		*rivers = new Rivers(mesh, maxHeight, 32.0f);
		(*rivers)->smooth(mesh);
		(*rivers)->carveInto(mesh, options.maxRiverGradient);
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

	void createRiverLists(const Mesh &mesh, const Rivers &rivers, Island::RiverVertexLists &lists) {
		int len = rivers.riverList().size();
		lists.reserve(len);
		std::unordered_set<int> used;
		for (int i = 0; i != len; ++i) {
			used.clear();
			auto list = std::make_shared<Island::VertexList>();
			list->reserve(rivers.riverList()[i]->vertices.size() << 1);
			for (const auto &j : rivers.riverList()[i]->vertices) {
				if (used.find(j.index) == used.end()) {
					list->emplace_back(mesh.vertices[j.index], j.flow);
					used.insert(j.index);
				}
			}
			lists.push_back(list);
		}
		smoothRiverLists(lists);
	}

	void lowerMesh(Mesh &mesh, float amount) {
		for (Vector3 &vert : mesh.vertices) {
			vert.z -= amount;
		}
	}

}

void Island::generateVegetationMap(VegetationMap &vege) const {
	Grid<Mesh::VertexAndNormal> grid(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
	lods[1].rasterize(grid);
	for (int y = 0, i = 0; y != grid.height(); ++y) {
		for (int x = 0; x != grid.width(); ++x) {
			vege.data()[i++] = computeAlbedo(grid.data()[i]);
		}
	}
}

void Island::generateNormalAndOcclusianMap(NormalAndOcclusianMap &nao) const {
	Grid<Mesh::VertexAndNormal> grid(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
	lods[1].rasterize(grid);
	Grid<Vector3> normals(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
	Grid<uint8_t> occlusian(MOTU_TEXTURE_MAP_SIZE, MOTU_TEXTURE_MAP_SIZE);
	for (int y = 0, i = 0; y != grid.height(); ++y) {
		for (int x = 0; x != grid.width(); ++x, ++i) {
			normals.data()[i] = grid.data()[i].normal;
			occlusian.data()[i] = computeOcclusian(grid, x, y);
		}
	}
	assignNormalAndOcclusianMap(nao, lods[2], normals, occlusian);
}

void Island::generateTopology(std::default_random_engine &rd, const Options &options) {
	Mesh preSlice[3];
	createInitialTerrain(rd, preSlice[2]);
	for (int i = 0; i != 2; ++i) {
		preSlice[2].tesselate();
		preSlice[2].smooth();
	}
	maxHeight = generateSeas(preSlice[2], rd, maxZ, options);
	preSlice[2].tesselate();
	preSlice[2].dirty();
	Rivers *rivers = erode(rd, preSlice[2], maxZ, options);
	preSlice[1] = preSlice[2];
	preSlice[1].dirty();
	preSlice[1].tesselate();
	erode(rd, &rivers/*, preSlice[2]*/, preSlice[1], maxZ, options, 2, [](Mesh &nw, int erosianPasses) {
		applyHydrolicErosian(nw, erosianPasses);
	});
	preSlice[0] = preSlice[1];
	preSlice[0].tesselate();
	//faceErode(preSlice[0]);
	preSlice[0].dirty();
	for (int i = 0; i != 8; ++i) {
		eatCoastlines(preSlice[0]);
	}
	lowerMesh(preSlice[0], maxZ * 0.01f);
	improveCliffs(preSlice[0]);
	formBeaches(preSlice[0]);
	preSlice[0].normals.clear();
	std::vector<float> richness;
	erode(rd, &rivers/*, preSlice[1]*/, preSlice[0], maxZ, options, 8, [&richness, &rd, this](Mesh &nw, int erosianPasses) {
		applyHydrolicErosian(rd, nw, erosianPasses, richness, mDecoration);
	});
	mSoilRichness = preSlice[0];
	for (size_t i = 0; i != richness.size(); ++i) {
		mSoilRichness.vertices[i].z = richness[i];
	}
	preSlice[0].tesselate();
	preSlice[0].dirty();
	finalRiverPass(&rivers, preSlice[0], maxZ, options);
	preSlice[0].smoothIfPositiveZ();
	preSlice[0].normals.clear();
	createRiverMeshes(*rivers, preSlice[0], mRiverMeshes);
	createRiverLists(preSlice[0], *rivers, mRiverVertexLists);
	correctLods(preSlice);
	for (int i = 0; i != 3; ++i) {
		preSlice[i].slice(BoundingBox(0.0f, 0.0f, -0.02f, 1.0f, 1.0f, 1.0f), lods[i]);
	}
	delete rivers;
	placeCoastalRocks(rd, lods[1], mDecoration);
	mDecoration.addRocks(rd, lods[0]);
	//mRivers = std::unique_ptr<Rivers>(rivers);
}