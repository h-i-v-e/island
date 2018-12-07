#include "decoration.h"
#include "mesh.h"
#include "z_axis_collider.h"
#include "mesh_triangle_map.h"
#include <unordered_map>

using namespace motu;

namespace {
	std::vector<Vector3> &copyOutVertices(const Mesh &in, const MeshEdgeMap &mem, const std::unordered_set<int> &ints, std::vector<Vector3> &out) {
		size_t len = ints.size();
		out.resize(len);
		size_t offset = 0;
		for (int i : ints) {
			Vector3 pos = in.vertices[i];
			for (auto j = mem.vertex(i); j.first != j.second; ++j.first) {
				float z = in.vertices[*j.first].z;
				if (z < pos.z) {
					pos.z = z;
				}
			}
			out[offset++] = pos;
		}
		return out;
	}

	std::vector<Vector3> &copyOutVertices(const Mesh &in, const std::unordered_set<int> &ints, std::vector<Vector3> &out) {
		size_t len = ints.size();
		out.resize(len);
		size_t offset = 0;
		for (int i : ints) {
			out[offset++] = in.vertices[i];
		}
		return out;
	}

	void removeUnderwater(const Mesh &in, std::unordered_set<int> &offsets) {
		for (int i : offsets) {
			if (in.vertices[i].z < 0.00025f) {
				offsets.erase(i);
			}
		}
	}

	Grid<float> &rasterize(const Mesh &mesh, const std::unordered_set<int> &offsets, Grid<float> &output) {
		Mesh buffer(mesh);
		for (size_t i = 0; i != buffer.vertices.size(); ++i) {
			buffer.vertices[i].z = offsets.find(i) == offsets.end() ? 0.0f : 1.0f;
		}
		buffer.rasterize(output);
	}

	Grid<float> &rasterize(const Mesh &mesh, const std::vector<float> &strengths, Grid<float> &output) {
		Mesh buffer(mesh);
		MeshEdgeMap mem(mesh);
		for (size_t i = 0; i != buffer.vertices.size(); ++i) {
			buffer.vertices[i].z = strengths[i];
		}
		buffer.rasterize(output);
	}
}

std::ostream& motu::operator<<(std::ostream &out, const Decoration &decoration) {
	writeOutSet(out, decoration.mTrees);
	writeOutSet(out, decoration.mBushes);
	writeOutSet(out, decoration.mRocks);
	writeOutVector(out, decoration.soilRichness);
	return out;
}

std::istream& motu::operator>>(std::istream &in, Decoration &decoration) {
	readInSet(in, decoration.mTrees);
	readInSet(in, decoration.mBushes);
	readInSet(in, decoration.mRocks);
	readInVector(in, decoration.soilRichness);
	return in;
}

void Decoration::computePositions(const Mesh &lod0, const MeshEdgeMap &mem) {
	removeUnderwater(lod0, mTrees);
	removeUnderwater(lod0, mBushes);
	copyOutVertices(lod0, mem, mTrees, treePositions);
	copyOutVertices(lod0, mBushes, bushPositions);
	copyOutVertices(lod0, mem, mRocks, rockPositions);
}

void Decoration::spreadSoilRichness(const Mesh &mesh, const MeshEdgeMap &mem) {
	size_t len = mesh.vertices.size(), from = soilRichness.size();
	soilRichness.resize(len);
	for (size_t i = from; i < len; ++i) {
		int count = 0;
		float total = 0.0f;
		for (auto k = mem.vertex(i); k.first != k.second; ++k.first) {
			int vert = *k.first;
			if (vert < from) {
				total += soilRichness[vert];
				++count;
			}
		}
		soilRichness[i] = count == 0 ? 0.0f : total / static_cast<float>(count);
	}
}

Grid<float> &Decoration::fillForestMap(const Mesh &mesh, Grid<float> &grid) const {
	return rasterize(mesh, mTrees, grid);
}

Grid<float> &Decoration::fillRockMap(const Mesh &mesh, Grid<float> &grid) const {
	return rasterize(mesh, mRocks, grid);
}

Grid<float> &Decoration::fillRiverMap(const Mesh &mesh, Grid<float> &grid) const {
	return rasterize(mesh, mRiver, grid);
}

Grid<float> &Decoration::fillSoilRichnessMap(const Mesh &mesh, Grid<float> &grid) const {
	return rasterize(mesh, soilRichness, grid);
}

void Decoration::normaliseSoilRichness() {
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
}