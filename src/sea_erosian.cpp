#include "sea_erosian.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"
#include "mesh.h"

#include <memory>
#include <limits>
#include <functional>

using namespace motu;

namespace {
	struct CoastMapper {
		const MeshEdgeMap *mep;
		//const MeshTriangleMap mtp;
		const Mesh *mesh;
		std::vector<bool> visited;
		//Coastlines *coasts;

		CoastMapper(const MeshEdgeMap &mep, const Mesh &mesh/*, Coastlines &coasts*/) : mep(&mep)/*, mtp(mesh)*/, mesh(&mesh), visited(mesh.vertices.size(), false)/*, coasts(&coasts)*/{
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

		int findSeaJoin(int last, int offset, int next) {
			auto ledges = mep->vertex(last);
			auto oedges = mep->vertex(offset);
			auto nedges = mep->vertex(next);
			while (ledges.first != ledges.second) {
				for (auto i = oedges.first; i != oedges.second; ++i) {
					if (*ledges.first == *i) {
						for (auto j = nedges.first; j != nedges.second; ++j) {
							if (*j == *i && mesh->vertices[*j].z < 0.0f) {
								return *j;
							}
						}
					}
				}
				++ledges.first;
			}
			return -1;
		}

		int getSeaRatio(int last, int offset, int next) {
			auto edges = mep->vertex(offset);
			int sea = 0;
			while (edges.first != edges.second) {
				int vert = *edges.first++;
				if (vert != last && vert != next) {
					if (mesh->vertices[vert].z < 0.0f) {
						++sea;
					}
					else {
						--sea;
					}
				}
			}
			return sea;
		}

		void followCoast(int offset, std::vector<std::pair<int, int>> &out, std::function<int(int, int, int)> setSecond) {
			out.emplace_back(offset, -1);
			visited[offset] = true;
			for (bool going = true; going;){
				going = false;
				auto edges = mep->vertex(offset);
				while (edges.first != edges.second) {
					int i = *edges.first++;
					if ((!visited[i]) && onCoast(i)) {
						out.emplace_back(i, -1);
						visited[i] = true;
						offset = i;
						going = true;
						break;
					}
					//++edges.first;
				}
			}
			if (out.size() < 3) {
				return;
			}
			for (int i = 2; i != out.size(); ++i) {
				out[i - 1].second = setSecond(out[i - 2].first, out[i - 1].first, out[i].first);
			}
		}

		void followCoastWithJoins(int offset, std::vector<std::pair<int, int>> &out) {
			followCoast(offset, out, [this](int last, int current, int next) {
				return findInlandJoin(last, current, next);
			});
		}

		void followCoastWithSeaJoins(int offset, std::vector<std::pair<int, int>> &out) {
			followCoast(offset, out, [this](int last, int current, int next) {
				return findSeaJoin(last, current, next);
			});
		}

		void followCoastWithSeaCount(int offset, std::vector<std::pair<int, int>> &out) {
			followCoast(offset, out, [this](int last, int current, int next) {
				return getSeaRatio(last, current, next);
			});
		}
		
		void followCoastAndSmooth(int offset, std::vector<std::pair<int, int>> &out, std::vector<Vector3> &smoothed) {
			followCoast(offset, out, [this, &smoothed](int last, int current, int next) {
				int ret = smoothed.size();
				smoothed.push_back((mesh->vertices[last] + mesh->vertices[current] + mesh->vertices[next]) / 3.0f);
				return ret;
			});
		}
		
		void findCoasts(Coastlines &coasts, std::function<void(int offset, std::vector<std::pair<int, int>> &out)> func) {
			for (int i = 0; i != mesh->vertices.size(); ++i) {
				if ((!visited[i]) && onCoast(i)) {
					coasts.emplace_back(std::make_unique<std::vector<std::pair<int, int>>> ());
					func(i, *coasts.back());
				}
			}
		}

		/*void findCoasts() {
			for (int i = 0; i != mesh->vertices.size(); ++i) {
				if ((!visited[i]) && onCoast(i)) {
					coasts->emplace_back(std::make_unique<std::vector<std::pair<int, int>>>());
					followCoast(i, *coasts->back());
				}
			}
		}*/
	};

	/*bool isCliff(const Vector2 &a, const Vector2 &b, const Vector2 &c, const Vector2 &land) {

	}*/


	float smooth(const MeshEdgeMap &mep, Mesh &mesh, int offset) {
		float total = mesh.vertices[offset].z;
		int count = 1;
		for (auto n = mep.vertex(offset); n.first != n.second; ++n.first, ++count) {
			total += mesh.vertices[*n.first].z;
		}
		return total / static_cast<float>(count);
	}

	void raiseSeaNeighbours(const MeshEdgeMap &mep, Mesh &mesh, int offset) {
		for (auto n = mep.vertex(offset); n.first != n.second; ++n.first) {
			int offset = *n.first;
			Vector3 &pos = mesh.vertices[offset];
			if (pos.z < 0.0f) {
				pos.z = 0.0f;
			}
		}
	}
}

void motu::mapCoastlines(const MeshEdgeMap &mep, const Mesh &mesh, Coastlines &coastlines) {
	CoastMapper coastMapper(mep, mesh);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &out) {
		coastMapper.followCoastWithJoins(offset, out);
	});
}

void motu::improveCliffs(Mesh &mesh) {
	improveCliffs(MeshEdgeMap(mesh), mesh);
}

void motu::improveCliffs(const MeshEdgeMap &mep, Mesh &mesh) {
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
				shift *= 0.95;
				mesh.vertices[j.second] += shift;
				mesh.vertices[j.first].z = 0.0f;
			}
		}
	}
}

void motu::applySeaErosian(Mesh &mesh, float strength) {
	applySeaErosian(MeshEdgeMap(mesh), mesh, strength);
}

void motu::applySeaErosian(const MeshEdgeMap &mep, Mesh &mesh, float strength) {
	Coastlines coastlines;
	CoastMapper coastMapper(mep, mesh);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &section) {
		coastMapper.followCoastWithSeaCount(offset, section);
	});
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			if (j.second > 0){
				float z = mesh.vertices[j.first].z;
				if (z < FLT_EPSILON) {
					mesh.vertices[j.first].z = -FLT_EPSILON;
					continue;
				}
				mesh.vertices[j.first].z = std::max(-FLT_EPSILON, z - static_cast<float>(j.second) * strength / z);
			}
		}
	}
}

void motu::eatCoastlines(const MeshEdgeMap &mep, Mesh &mesh) {
	Coastlines coastlines;
	CoastMapper coastMapper(mep, mesh);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &section) {
		coastMapper.followCoastWithSeaJoins(offset, section);
	});
	for (auto i = coastlines.begin(); i != coastlines.end(); ++i) {
		for (auto j = (*i)->begin(); j != (*i)->end(); ++j) {
			if (j->second != -1) {
				mesh.vertices[j->first].z = mesh.vertices[j->second].z;
			}
		}
	}
}

void motu::smoothCoastlines(const MeshEdgeMap &mep, Mesh &mesh) {
	CoastMapper coastMapper(mep, mesh);
	Coastlines coastlines;
	std::vector<Vector3> smooth;
	coastMapper.findCoasts(coastlines, [&coastMapper, &smooth](int offset, std::vector<std::pair<int, int>> &out) {
		coastMapper.followCoastAndSmooth(offset, out, smooth);
	});
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			mesh.vertices[j.first] = smooth[j.second];
		}
	}
}

void motu::formBeaches(const MeshEdgeMap &mep, Mesh &mesh) {
	Coastlines coastlines;
	CoastMapper coastMapper(mep, mesh);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &section) {
		coastMapper.followCoastWithSeaCount(offset, section);
	});
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			if (j.second < 0) {
				float z = smooth(mep, mesh, j.first);
				mesh.vertices[j.first].z = z;
				for (auto v = mep.vertex(j.first); v.first != v.second; ++v.first) {
					int offset = *v.first;
					if (!coastMapper.onCoast(offset)) {
						mesh.vertices[offset].z = z;
					}
				}
				mesh.vertices[j.first].z = smooth(mep, mesh, j.first);
			}
		}
	}
}