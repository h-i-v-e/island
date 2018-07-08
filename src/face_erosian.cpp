#include "face_erosian.h"
#include "mesh.h"
#include "rtree.h"
#include "triangle.h"
#include "graph.h"

using namespace motu;

namespace {
	typedef Graph<int> Voronoi;

	void track(Voronoi &voronoi, Voronoi::Offsets face, Mesh &mesh) {
		int lowest = *face.first;
		float lowestZ = mesh.vertices[voronoi.vertices[lowest]].z;
		while (++face.first != face.second) {
			int current = *face.first;
			float z = mesh.vertices[voronoi.vertices[current]].z;
			if (z <= 0.0f) {
				return;
			}
			if (z < lowestZ) {
				lowest = current;
				lowestZ = z;
			}
		}
		float speed = 0.0f;
		for (;;) {
			int next = -1;
			auto neighbours = voronoi.neighbours(lowest);
			while (neighbours.first != neighbours.second) {
				int test = *neighbours.first++;
				float z = mesh.vertices[voronoi.vertices[test]].z;
				if (z < 0.0f) {
					return;
				}
				if (z < lowestZ) {
					next = test;
					lowestZ = z;
				}
			}
			if (next == -1) {
				mesh.vertices[voronoi.vertices[lowest]].z += speed;
				return;
			}
			float carrying = speed;
			speed += mesh.vertices[voronoi.vertices[lowest]].z - lowestZ;
			speed *= 0.5f;
			mesh.vertices[voronoi.vertices[lowest]].z += carrying - speed;
			lowest = next;
		}
	}

	void buildVoronoi(Voronoi &voronoi, const Mesh &mesh, std::vector<int> &centroids) {
		GraphBuilder<int> graphBuilder(voronoi);
		const MeshTriangleMap &mtm = mesh.triangleMap();
		std::vector<int> faceBuffer;
		for (size_t i = 0; i != mesh.vertices.size(); ++i) {
			faceBuffer.clear();
			for (auto j = mtm.vertex(i); j.first != j.second; ++j.first) {
				faceBuffer.push_back(centroids[*j.first]);
			}
			graphBuilder.addPolygon(faceBuffer.begin(), faceBuffer.end());
		}
		graphBuilder.complete();
	}

	float getFaceHeight(const Voronoi &voro, Voronoi::Offsets &face, const Mesh &mesh) {
		float total = 0.0f;
		float size = static_cast<float>(face.second - face.first);
		while (face.first != face.second) {
			total += mesh.vertices[voro.vertices[*face.first++]].z;
		}
		return total / size;
	}
}


void motu::faceErode(Mesh &mesh) {
	Mesh tesselated;
	std::vector<int> centroids;
	mesh.tesselateAndMapCentroids(centroids, tesselated);
	Voronoi voronoi;
	buildVoronoi(voronoi, mesh, centroids);
	std::vector<std::pair<int, float>> sorted(voronoi.numFaces());
	for (int i = 0, j = static_cast<int>(sorted.size()); i != j; ++i) {
		sorted[i] = std::make_pair(i, getFaceHeight(voronoi, voronoi.face(i), tesselated));
	}
	std::sort(sorted.begin(), sorted.end(), [](const std::pair<int, float> &a, const std::pair<int, float> &b) {
		return a.second > b.second;
	});
	for (auto i = sorted.begin(); i != sorted.end() && i->second > 0.0f; ++i) {
		track(voronoi, voronoi.face(i->first), tesselated);
	}
	tesselated.smooth();
	mesh = tesselated;
	mesh.dirty();
}