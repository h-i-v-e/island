#include "decoration.h"
#include "mesh.h"
#include "z_axis_collider.h"
#include "mesh_triangle_map.h"
#include <unordered_map>

using namespace motu;

namespace {
	std::vector<Vector3> &copyOutVertices(const Mesh &in, const MeshEdgeMap &mem, const std::vector<int> &ints, std::vector<Vector3> &out) {
		size_t len = ints.size();
		out.resize(len);
		for (size_t i = 0; i != len; ++i) {
			int vert = ints[i];
			Vector3 pos = in.vertices[vert];
			for (auto j = mem.vertex(vert); j.first != j.second; ++j.first) {
				float z = in.vertices[*j.first].z;
				if (z < pos.z) {
					pos.z = z;
				}
			}
			out[i] = pos;
		}
		return out;
	}

	std::vector<Vector3> &copyOutVertices(const Mesh &in, const std::vector<int> &ints, std::vector<Vector3> &out) {
		size_t len = ints.size();
		out.resize(len);
		for (size_t i = 0; i != len; ++i) {
			out[i] = in.vertices[ints[i]];
		}
		return out;
	}
}

std::vector<Vector3> &Decoration::getTrees(const Mesh &lod0, const MeshEdgeMap &mem, std::vector<Vector3> &out) const{
	return copyOutVertices(lod0, mem, trees, out);
}

std::vector<Vector3> &Decoration::getBushes(const Mesh &lod0, std::vector<Vector3> &out) const{
	return copyOutVertices(lod0, bushes, out);
}

std::vector<Vector3> &Decoration::getRocks(const Mesh &lod0, const MeshEdgeMap &mem, std::vector<Vector3> &out) const{
	return copyOutVertices(lod0, mem, rocks, out);
}

std::ostream& motu::operator<<(std::ostream &out, const Decoration &decoration) {
	writeOutVector(out, decoration.forest);
	writeOutVector(out, decoration.bushes);
	writeOutVector(out, decoration.rocks);
	out << decoration.mesh;
	writeOutVector(out, decoration.forest);
	writeOutVector(out, decoration.soilRichness);
	return out;
}

std::istream& motu::operator>>(std::istream &in, Decoration &decoration) {
	readInVector(in, decoration.forest);
	readInVector(in, decoration.bushes);
	readInVector(in, decoration.rocks);
	in >> decoration.mesh;
	readInVector(in, decoration.forest);
	readInVector(in, decoration.soilRichness);
	return in;
}