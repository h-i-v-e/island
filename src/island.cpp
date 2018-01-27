#include <stack>
#include <limits>
#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "island.h"
#include "noise_layer.h"
//#include "triangulated_terrain_graph.h"
#include "bounding_box.h"
#include "terrain.h"
#include "hydrolic_erosian.h"
#include "mesh_edge_map.h"
#include "rivers.h"

using namespace motu;

namespace {
	short occlusianCursor[30] = {
		-8, -2, -4, 2, -2, 1, -1, 1,
		-1, -1, -1, -4, 1, 2, 1, 1,
		1, -1, 1, -2, 2, 4, 2, -1,
		2, -8, 4, -2, 8, 2
	};

	/*struct FaceData {
		bool sea;
	};

	struct VertexData;

	typedef typename motu::HalfEdge<Vector3, FaceData, VertexData> HalfEdge;
	typedef typename HalfEdge::Vertex Vertex;
	typedef typename HalfEdge::Face Face;
	typedef typename IterableObjectPool<HalfEdge> HalfEdges;
	typedef typename IterableObjectPool<Vertex> Vertices;
	typedef typename IterableObjectPool<Face> Faces;

	struct VertexData {
		Vector3 normal;
		float z, seaDistance, landDistance;
		int flow;
		Vertex *down;
		bool cliff;

		VertexData() : z(0.0f), flow(0), down(nullptr), cliff(false) {}
	};*/

	void MapSea(const Terrain::HalfEdge &perimeter, const std::vector<bool> &data, std::unordered_set<int> &sea) {
		std::stack<const Terrain::Face*> unvisited;
		unvisited.push(&perimeter.face());
		while (!unvisited.empty()) {
			const Terrain::Face *next(unvisited.top());
			unvisited.pop();
			for (const Terrain::HalfEdge &edge : next->halfEdges()) {
				if (edge.pair) {
					Terrain::Face *face(&edge.pair->face());
					if (data[face->data()] && sea.find(face->data()) == sea.end()) {
						sea.insert(face->data());
						unvisited.push(face);
					}
				}
			}
		}
	}

	void RemoveLakes(std::vector<bool> &data, Terrain::Faces &faces, Terrain::Face &externalFace) {
		std::unordered_set<int> sea;
		MapSea(*externalFace.halfEdge(), data, sea);
		for (auto i = faces.begin(); i != faces.end(); ++i) {
			if (data[i->data()]) {
				if (sea.find(i->data()) == sea.end()) {
					data[i->data()] = false;
				}
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
	float ComputeDistanceTo(Terrain &terrain, const std::vector<bool> &sea, std::vector<float> &distanceVec, DistanceTo dt) {
		typedef std::stack<Terrain::Vertex*> Stack;

		Stack circles[2];
		Stack *last = circles;
		Stack *next = circles + 1;

		float distance = 0.0f, add = 0.5f, addMul = 1.05f;
		for (auto i = terrain.vertices().begin(); i != terrain.vertices().end(); ++i) {
			//dt.target(i->data()) = std::numeric_limits<float>::max();
			for (auto j : i->inbound()) {
				if (dt(sea[j.face().data()])) {
					//dt.target(i->data()) = 0;
					distanceVec[i->data()] = 0.0f;
					last->push(&*i);
					break;
				}
			}
		}
		do {
			add *= addMul;
			distance += add;
			while (!last->empty()) {
				Terrain::Vertex *vertex(last->top());
				last->pop();
				for (auto j = vertex->inbound().begin(); j != vertex->inbound().end(); ++j) {
					if (j->pair && distance < distanceVec[j->pair->vertex().data()]) {
						distanceVec[j->pair->vertex().data()] = distance;
						next->push(&j->pair->vertex());
					}
				}
			}
			Stack *swap = last;
			last = next;
			next = swap;
		} while (!last->empty());
		return distance;
	}

	void setZValues(Terrain::Vertices &vertices, const std::vector<float> &distanceToSea, const std::vector<float> &distanceToLand, float maxDistance, float maxZ) {
		float mul = maxZ / maxDistance;
		for (auto i = vertices.begin(); i != vertices.end(); ++i) {
			i->position().z = (distanceToSea[i->data()] - distanceToLand[i->data()]) * mul;
			if (i->position().z < -0.002f) {
				i->position().z = -0.002f;
			}
		}
	}

	float generateSeas(Terrain &terrain, std::default_random_engine &rnd, float &maxZ, float waterRatio) {
		std::uniform_real_distribution<float> dis(0.0f, 1.0f);
		maxZ = std::normal_distribution<float>(maxZ, maxZ * 0.2f)(rnd);
		std::vector<bool> seas;
		std::vector<float> distanceToSea, distanceToLand;
		terrain.initFaceData(seas, false);
		terrain.initVertexData(distanceToSea, std::numeric_limits<float>::max());
		terrain.initVertexData(distanceToLand, std::numeric_limits<float>::max());
		NoiseLayer layers[4];
		float strength = 2.0f;
		for (int i = 0; i != 4; ++i) {
			layers[i] = NoiseLayer(Vector2(dis(rnd), dis(rnd)), strength);
			strength *= 4.0f;
		}
		int seaCount = 0;
		for (auto i = terrain.faces().begin(); i != terrain.faces().end(); ++i) {
			if (&*i == &terrain.externalFace()) {
				continue;
			}
			Vector3 centre = i->calculateCentroid();
			centre.z = 0.0f;
			float noise = 0.0f;
			float dilute = 1.0f;
			for (int i = 0; i != 4; ++i) {
				noise += layers[i].get(centre.toVector2()) / dilute;
				dilute *= 2.0f;
			}
			noise *= 1.0f - ((centre - Vector3(0.5f, 0.5f, 0.0f)).magnitude() * 1.5f);
			if (seas[i->data()] = noise < waterRatio) {
				++seaCount;
			}
		}
		RemoveLakes(seas, terrain.faces(), terrain.externalFace());
		float maxHeight = ComputeDistanceTo(terrain, seas, distanceToSea, DistanceToSea());
		ComputeDistanceTo(terrain, seas, distanceToLand, DistanceToLand());

		setZValues(terrain.vertices(), distanceToSea, distanceToLand, maxHeight, maxZ);
		return maxHeight;
	}

	void generateLod0(const Mesh &lod1, Mesh &lod0) {
		Mesh mid;
		lod1.tesselate(mid);
		mid.tesselate(lod0);
		lod0.smooth();
		//lod0 = lod1;
	}

	uint32_t computeOcclusian(const Grid<Mesh::VertexAndNormal> &grid, int x, int y) {
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
		return count > 0 ? (static_cast<uint32_t>((1.0f - (total / count)) * 255.0f) << 24) : 0xff000000;
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

	/*void pushDownPerimiter(TerrainGraph &graph) {
		for (auto i = graph.externalFace().halfEdges().begin(); i != graph.externalFace().halfEdges().end(); ++i) {
			i->vertex().data().z = -1.0f;
		}
	}*/

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

	void correctLodsAndGenerateMaps(Mesh *lods, Island::NormalAndOcclusianMap &normalMap) {
		Grid<Mesh::VertexAndNormal> grid(2048, 2048);
		lods[0].calculateNormals();
		lods[0].rasterize(grid);
		for (size_t y = 0, i = 0; y != grid.height(); ++y) {
			for (size_t x = 0; x != grid.width(); ++x, ++i) {
				normalMap.data()[i] = createNormalValue(grid.data()[i].normal) | computeOcclusian(grid, x, y);
			}
		}
		correctZ(grid, lods[1]);
		correctZ(grid, lods[2]);
	}

	Vector3 vector2ToVector3(const Vector2 &vec) {
		return Vector3(vec.x, vec.y, 0.0f);
	}

	void createSeedPoints(std::default_random_engine &rd, int number, int relaxations, std::vector<Vector2> &points) {
		points.reserve(number);
		std::uniform_real_distribution<float> dis(0.0f, 1.0f);
		for (int i = 0, j = number - 8; i < j; ++i) {
			points.emplace_back(dis(rd), dis(rd));
		}
		points.emplace_back(0.0f, 0.0f);
		points.emplace_back(0.0f, 0.5f);
		points.emplace_back(0.0f, 1.0f);
		points.emplace_back(0.5f, 1.0f);
		points.emplace_back(1.0f, 1.0f);
		points.emplace_back(1.0f, 0.5f);
		points.emplace_back(1.0f, 0.0f);
		points.emplace_back(0.5f, 0.0f);
		while (relaxations-- > 0) {
			Terrain::relax(points);
		}
	}

	Terrain *createInitialTerrain(std::default_random_engine &rd) {
		std::vector<Vector2> points;
		createSeedPoints(rd, 512, 10, points);
		return new Terrain(points, 2);
	}

	Terrain *replaceWithSmoothed(Terrain *terrain, int iterations) {
		Mesh meshes[2];
		terrain->toMesh(meshes[iterations & 1]);
		while (iterations > 0) {
			meshes[iterations & 1].tesselate(meshes[(iterations ^ 1) & 1]);
			--iterations;
		}
		meshes->smooth();
		return new Terrain(*meshes);
	}

	void tesselateAndSmooth(const Mesh &in, Mesh &out) {
		Mesh buffer;
		in.tesselate(buffer);
		buffer.tesselate(out);
		out.smooth();
	}

	Rivers *erode(Mesh &mesh) {
		MeshEdgeMap mep(mesh);
		applyHydrolicErosian(mesh, mep, 16);
		Rivers *rivers = new Rivers(mesh, mep);
		rivers->carveInto(mesh, mep, 0.002f);
		return rivers;
	}

	void erode(const Rivers &river, const Mesh &old, Mesh &nw) {
		MeshEdgeMap mep(nw);
		applyHydrolicErosian(nw, mep, 16);
		Rivers rivers(river, old, nw, mep);
		rivers.smooth(nw, mep);
		rivers.carveInto(nw, mep, 0.001f);
	}
}

void Island::generateTopology(std::default_random_engine &rd, float waterRatio) {
	Terrain *terrain = createInitialTerrain(rd);
	maxHeight = generateSeas(*terrain, rd, maxZ, waterRatio);
	terrain = replaceWithSmoothed(terrain, 1);
	terrain->toMesh(lods[2]);
	Rivers *rivers = erode(lods[2]);
	lods[2].tesselate(lods[1]);
	erode(*rivers, lods[2], lods[1]);
	delete rivers;
	tesselateAndSmooth(lods[1], lods[0]);
	correctLodsAndGenerateMaps(lods, mNormalAndOcclusianMap);

	/*Mesh mesh;
	createInitialMesh(rd, 1024, mesh);
	HalfEdges halfEdges(mesh.triangles.size());
	Vertices vertices(mesh.vertices.size());
	Faces faces(mesh.triangles.size() / 3);
	Face externalFace = mesh.createHalfEdgeGraph(vertices, halfEdges, faces);
	generateSeas(faces, vertices, &externalFace, rd, maxZ, waterRatio);
	

	//maxHeight = generateSeas(graph, rd, maxZ, waterRatio);
	TriangulatedTerrainGraph firstPass(graph);
	Mesh raw[3];
	firstPass.smooth();
	firstPass.erode(8, 0.04f);
	firstPass.toMesh(raw[2]);
	TriangulatedTerrainGraph::RiverSections riverSections;
	TriangulatedTerrainGraph::LastList lastList;
	firstPass.fillLastList(lastList, 3.0f);
	lastList.copyToRiverSections(riverSections);
	TriangulatedTerrainGraph tri(firstPass);
	tri.erode(8, 0.02f);
	lastList.clear();
	tri.fillLastListInterpolated(riverSections, lastList);
	tri.carveRiverBeds(lastList);
	tri.smoothRiverBeds(lastList);
	tri.generateRiverMesh(lastList, mRivers);
	tri.toMesh(raw[1]);
	generateLod0(raw[1], raw[0]);
	BoundingBox box(0.0f, 0.0f, -1.0f, 1.0f, 1.0f, 1.0f);
	for (size_t i = 0; i != 3; ++i) {
		raw[i].slice(box, lods[i]);
	}
	correctLodsAndGenerateMaps(lods, mNormalAndOcclusianMap);*/
}