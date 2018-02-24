#include "sea_erosian.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"
#include "mesh.h"

#include <memory>
#include <limits>

using namespace motu;

namespace {
	struct CoastMapper {
		const MeshEdgeMap *mep;
		//const MeshTriangleMap mtp;
		const Mesh *mesh;
		std::vector<bool> visited;
		Coastlines *coasts;

		CoastMapper(const MeshEdgeMap &mep, const Mesh &mesh, Coastlines &coasts) : mep(&mep)/*, mtp(mesh)*/, mesh(&mesh), visited(mesh.vertices.size(), false), coasts(&coasts){
		}

		bool onCoast(int offset) const{
			if (mesh->vertices[offset].z < 0.0f) {
				return false;
			}
			auto edges = mep->vertex(offset);
			while (edges.first != edges.second) {
				if (mesh->vertices[*edges.first].z < 0.0f) {
					return true;
				}
				++edges.first;
			}
			return false;
		}

		int landCount(int offset, int triangle) {
			int count = 0;
			for (int i = 0; i != 3; ++i) {
				int o = mesh->triangles[triangle + i];
				if (o == offset) {
					continue;
				}
				if (mesh->vertices[o].z >= 0.0f) {
					++count;
				}
			}
			return count;
		}

		/*int findInlandTriangle (int offset) {
			int best = -1, idx;
			auto triangles = mtp.vertex(offset);
			while (triangles.first != triangles.second) {
				int count = landCount(offset, *triangles.first);
				if (count > best) {
					idx = *triangles.first;
					best = count;
				}
				++triangles.first;
			}
			return idx;
		}*/

		int findInlandJoin(int last, int offset, int next) {
			auto ledges = mep->vertex(last);
			auto oedges = mep->vertex(offset);
			auto nedges = mep->vertex(next);
			while (ledges.first != ledges.second) {
				for (auto i = oedges.first; i != oedges.second; ++i) {
					if (*ledges.first == *i) {
						for (auto j = nedges.first; j != nedges.second; ++j) {
							if (*j == *i && mesh->vertices[*j].z > 0.0f) {
								return *j;
							}
						}
					}
				}
				++ledges.first;
			}
			return -1;
		}

		void followCoast(int offset, std::vector<std::pair<int, int>> &out) {
			out.emplace_back(offset, /*findInlandTriangle(offset)*/-1);
			visited[offset] = true;
			for (bool going = true; going;){
				going = false;
				auto edges = mep->vertex(offset);
				while (edges.first != edges.second) {
					int i = *edges.first;
					if ((!visited[i]) && onCoast(i)) {
						out.emplace_back(i, /*findInlandTriangle(offset)*/-1);
						visited[i] = true;
						offset = i;
						going = true;
						break;
					}
					++edges.first;
				}
			}
			if (out.size() < 3) {
				return;
			}
			for (int i = 2; i != out.size(); ++i) {
				out[i - 1].second = findInlandJoin(out[i - 2].first, out[i - 1].first, out[i].first);
			}
		}

		void findCoasts() {
			for (int i = 0; i != mesh->vertices.size(); ++i) {
				if ((!visited[i]) && onCoast(i)) {
					coasts->emplace_back(std::make_unique<std::vector<std::pair<int, int>>>());
					followCoast(i, *coasts->back());
				}
			}
		}
	};

	/*bool isCliff(const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &land) {

	}*/
}

void motu::mapCoastlines(const MeshEdgeMap &mep, const Mesh &mesh, Coastlines &coastlines) {
	CoastMapper(mep, mesh, coastlines).findCoasts();
}

void motu::applySeaErosian(Mesh &mesh) {
	applySeaErosian(MeshEdgeMap(mesh), mesh);
}

void motu::applySeaErosian(const MeshEdgeMap &mep, Mesh &mesh) {
	Coastlines coastlines;
	mapCoastlines(mep, mesh, coastlines);
	/*for (const auto &i : coastlines) {
		int lastTriangle = -1, lastVert;
		for (auto j : *i) {
			if (j.second == lastTriangle) {
				int inside;
				Vector2 total = Vector2::zero();
				for (int k = 0; k != 3; ++k) {
					int vert = mesh.triangles[j.second + k];
					if (vert != j.first && vert != lastVert) {
						inside = vert;
					}
					total += mesh.vertices[vert].asVector2();
				}
				total /= 3.0f;
				mesh.vertices[inside].x = total.x;
				mesh.vertices[inside].y = total.y;
				for (int k = 0; k != 3; ++k) {
					int vert = mesh.triangles[j.second + k];
					if (vert != inside) {
						mesh.vertices[vert].z = 0.0f;
					}
				}
			}
			lastTriangle = j.second;
			lastVert = j.first;
		}
	}*/
	for (const auto &i : coastlines) {
		for (auto j : *i) {
			if (j.second != -1) {
				Vector3 shift = mesh.vertices[j.first] - mesh.vertices[j.second];
				shift.z = 0.0f;
				shift *= 0.75;
				mesh.vertices[j.second] += shift;
				mesh.vertices[j.first].z = 0.0f;
			}
		}
	}
}