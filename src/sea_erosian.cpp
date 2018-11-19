#include "sea_erosian.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"
#include "mesh_utils.h"
#include "decoration.h"

#include <memory>
#include <limits>
#include <functional>

#ifndef MOTU_BEACH_HEIGHT
#define MOTU_BEACH_HEIGHT 0.0002
#endif

using namespace motu;

namespace {
	struct CoastMapper {
		const Mesh *mesh;
		std::vector<bool> visited;

		CoastMapper(const Mesh &mesh) : mesh(&mesh), visited(mesh.vertices.size(), false){
		}

		void clear() {
			std::fill(visited.begin(), visited.end(), false);
		}

		bool onCoast(int offset) const{
			if (mesh->vertices[offset].z < 0.0f) {
				return false;
			}
			auto edges = mesh->edgeMap().vertex(offset);
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
			const MeshEdgeMap &mep = mesh->edgeMap();
			auto ledges = mep.vertex(last);
			auto oedges = mep.vertex(offset);
			auto nedges = mep.vertex(next);
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
			const MeshEdgeMap &mep = mesh->edgeMap();
			auto ledges = mep.vertex(last);
			auto oedges = mep.vertex(offset);
			auto nedges = mep.vertex(next);
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
			auto edges = mesh->edgeMap().vertex(offset);
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
			const MeshEdgeMap &mep = mesh->edgeMap();
			for (bool going = true; going;){
				going = false;
				auto edges = mep.vertex(offset);
				while (edges.first != edges.second) {
					int i = *edges.first++;
					if ((!visited[i]) && onCoast(i)) {
						out.emplace_back(i, -1);
						visited[i] = true;
						offset = i;
						going = true;
						break;
					}
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


	float smooth(Mesh &mesh, int offset) {
		float total = mesh.vertices[offset].z;
		int count = 1;
		for (auto n = mesh.edgeMap().vertex(offset); n.first != n.second; ++n.first, ++count) {
			total += mesh.vertices[*n.first].z;
		}
		return total / static_cast<float>(count);
	}

	void raiseSeaNeighbours(Mesh &mesh, int offset) {
		for (auto n = mesh.edgeMap().vertex(offset); n.first != n.second; ++n.first) {
			int offset = *n.first;
			Vector3 &pos = mesh.vertices[offset];
			if (pos.z < 0.0f) {
				pos.z = 0.0f;
			}
		}
	}

	int findHighestNeighbour(Mesh &mesh, int offset) {
		float highest = mesh.vertices[offset].z;
		int idx = -1;
		for (auto n = mesh.edgeMap().vertex(offset); n.first != n.second; ++n.first) {
			float z = mesh.vertices[*n.first].z;
			if (z > highest) {
				highest = z;
				idx = *n.first;
			}
		}
		return idx;
	}

	int findLowestNeighbour(Mesh &mesh, int offset) {
		float lowest = mesh.vertices[offset].z;
		int idx = -1;
		for (auto n = mesh.edgeMap().vertex(offset); n.first != n.second; ++n.first) {
			float z = mesh.vertices[*n.first].z;
			if (z < lowest) {
				lowest = z;
				idx = *n.first;
			}
		}
		return idx;
	}

	float findLowestNeighbourZ(Mesh &mesh, int offset) {
		float lowest = mesh.vertices[offset].z;
		for (auto n = mesh.edgeMap().vertex(offset); n.first != n.second; ++n.first) {
			float z = mesh.vertices[*n.first].z;
			if (z < lowest) {
				lowest = z;
			}
		}
		return lowest;
	}

	float GetRockChance(const Mesh &mesh, int offset) {
		return (1.0f - mesh.normals[offset].dot(Vector3::unitZ()));
	}

	void forceMinimumDepth(Mesh &mesh, float depth) {
		for (Vector3 &vert : mesh.vertices) {
			if (vert.z < 0.0f && vert.z > depth) {
				vert.z = depth;
			}
		}
	}

	struct IsSea {
		const Mesh &mesh;

		IsSea(const Mesh &mesh) : mesh(mesh){}

		bool operator()(int i) const {
			return mesh.vertices[i].z >= 0.0f;
		}
	};
}

void motu::mapCoastlines(const Mesh &mesh, Coastlines &coastlines) {
	CoastMapper coastMapper(mesh);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &out) {
		coastMapper.followCoastWithJoins(offset, out);
	});
}

void motu::improveCliffs(Mesh &mesh, std::unordered_set<int> &cliffs) {
	Coastlines coastlines;
	mapCoastlines(mesh, coastlines);
	for (const auto &i : coastlines) {
		for (auto j : *i) {
			if (j.second != -1) {
				/*Vector3 shift = mesh.vertices[j.first] - mesh.vertices[j.second];
				shift.z = 0.0f;
				shift *= 0.95;
				mesh.vertices[j.second] += shift;
				mesh.vertices[j.first].z = mesh.vertices;*/
				int idx = findLowestNeighbour(mesh, j.first);
				if (idx != -1) {
					Vector3 shift = mesh.vertices[j.first] - mesh.vertices[idx];
					shift.z = 0.0f;
					shift *= 0.95f;
					mesh.vertices[j.first] += shift;
					mesh.vertices[j.first].z = mesh.vertices[j.second].z;
					cliffs.insert(j.first);
					cliffs.insert(j.second);
					cliffs.insert(idx);
				}
			}
		}
	}
}

void motu::applySeaErosian(Mesh &mesh, float strength) {
	Coastlines coastlines;
	CoastMapper coastMapper(mesh);
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

void motu::eatCoastlines(Mesh &mesh, int steps) {
	Coastlines coastlines;
	CoastMapper coastMapper(mesh);
	for (;;) {
		coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &section) {
			coastMapper.followCoastWithSeaJoins(offset, section);
		});
		for (auto i = coastlines.begin(); i != coastlines.end(); ++i) {
			for (auto j = (*i)->begin(); j != (*i)->end(); ++j) {
				if (j->second != -1) {
					/*if (mesh.vertices[j->second].z >= 0.0f) {
						exit(1);
					}*/
					//mesh.vertices[j->first].z = mesh.vertices[j->second].z;
					float z = findLowestNeighbourZ(mesh, j->second);
					mesh.vertices[j->first].z = z;
					mesh.vertices[j->second].z = z;
				}
			}
		}
		if (--steps > 0) {
			coastMapper.clear();
			coastlines.clear();
		}
		else {
			std::vector<float> distance(mesh.vertices.size(), std::numeric_limits<float>::max());
			float max = computeDistance(IsSea(mesh), mesh, mesh.edgeMap(), distance);
			std::cout << "Max distance: " << max << std::endl;
			float min = 0.0f;
			for (const Vector3 &vec : mesh.vertices) {
				float val = vec.z;
				if (val < min) {
					min = val;
				}
			}
			std::cout << "Min: " << min << std::endl;
			float stepSize = min / max;
			std::cout << "Step: " << stepSize << std::endl;
			for (size_t i = 0, j = mesh.vertices.size(); i != j; ++i) {
				float dist = distance[i];
				if (dist > 0.0f) {
					mesh.vertices[i].z = dist * stepSize;
				}
			}
			mesh.dirty();
			return;
		}
	}
}

void motu::smoothCoastlines(Mesh &mesh) {
	CoastMapper coastMapper(mesh);
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

void motu::formBeaches(Mesh &mesh) {
	Coastlines coastlines;
	CoastMapper coastMapper(mesh);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &section) {
		coastMapper.followCoastWithSeaJoins(offset, section);
	});
	const MeshEdgeMap &mep = mesh.edgeMap();
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			if (j.second != -1) {
				int idx = findHighestNeighbour(mesh, j.first);
				if (idx == -1) {
					continue;
				}
				float z = mesh.vertices[j.first].z;
				mesh.vertices[idx].z = z;
				mesh.vertices[j.first].z = (z + mesh.vertices[j.second].z) * 0.5f;
			}
		}
	}
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			for (auto k = mep.vertex(j.first); k.first != k.second; ++k.first) {
				Vector3 &vert = mesh.vertices[*k.first];
				if (vert.z < 0.0f) {
					vert.z = MOTU_BEACH_HEIGHT;
				}
			}
		}
	}
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			for (auto k = mep.vertex(j.first); k.first != k.second; ++k.first) {
				for (auto l = mep.vertex(*k.first); l.first != l.second; ++l.first) {
					Vector3 &vert = mesh.vertices[*l.first];
					if (vert.z < 0.0f) {
						vert.z = 0;
					}
				}
			}
		}
	}
}

void motu::placeCoastalRocks(std::default_random_engine &rd, Decoration &decoration) {
	const MeshEdgeMap &mep = decoration.mesh.edgeMap();
	Coastlines coastlines;
	CoastMapper coastMapper(decoration.mesh);
	//std::unordered_set<int> rocked;
	std::uniform_real_distribution<float> full(0.0f, 1.0f);
	std::uniform_real_distribution<float> half(0.5f, 1.0f);
	std::uniform_real_distribution<float> quarter(0.25f, 0.5f);
	std::uniform_real_distribution<float> eigth(0.125f, 0.25f);
	coastMapper.findCoasts(coastlines, [&coastMapper](int offset, std::vector<std::pair<int, int>> &section) {
		coastMapper.followCoastWithSeaCount(offset, section);
	});
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			if (j.second > 0) {
				if (decoration.occupied.find(j.first) == decoration.occupied.end() && full(rd) < GetRockChance(decoration.mesh, j.first)) {
					decoration.occupied.insert(j.first);
					decoration.bigRocks.push_back(decoration.mesh.vertices[j.first].asVector2());
					//rocked.insert(j.first);
				}
			}
		}
	}
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			if (j.second > 0) {
				const Vector2 &middle = decoration.mesh.vertices[j.first].asVector2();
				auto neighbours = mep.vertex(j.first);
				while (neighbours.first != neighbours.second) {
					int offset = *neighbours.first++;
					const Vector2 &neighbour = decoration.mesh.vertices[offset].asVector2();
					if (decoration.occupied.find(offset) == decoration.occupied.end()) {
						if (full(rd) > GetRockChance(decoration.mesh, offset)) {
							continue;
						}
						Vector2 shift = neighbour - middle;
						if (!coastMapper.onCoast(offset) && decoration.mesh.vertices[offset].z > 0.0f) {
							shift *= half(rd);
							decoration.bigRocks.push_back(middle + shift);
						}
						else {
							shift *= quarter(rd);
							decoration.mediumRocks.push_back(middle + shift);
						}
						decoration.occupied.insert(offset);
					}
				}
			}
		}
	}
	for (const auto &i : coastlines) {
		for (const auto &j : *i) {
			if (j.second > 0) {
				auto neighbours = mep.vertex(j.first);
				while (neighbours.first != neighbours.second) {
					int offset = *neighbours.first++;
					const Vector2 &neighbour = decoration.mesh.vertices[offset].asVector2();
					auto neighboursNeighbours = mep.vertex(offset);
					while (neighboursNeighbours.first != neighboursNeighbours.second) {
						offset = *neighboursNeighbours.first++;
						if (decoration.occupied.find(offset) == decoration.occupied.end()) {
							const Vector2 &pos = decoration.mesh.vertices[offset].asVector2();
							decoration.smallRocks.push_back(neighbour + ((pos - neighbour) * eigth(rd)));
							decoration.occupied.insert(offset);
						}
					}
				}
			}
		}
	}
}