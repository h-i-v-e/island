#include "lake.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"
#include "mesh.h"

#include <unordered_map>

using namespace motu;

std::unordered_set<int> &Lake::fillLakeVertexSet(const Lake::Lakes &lakes, std::unordered_set<int> &out) {
	int count = 0;
	for (const Lake::Ptr &lake : lakes) {
		count += static_cast<int>(lake->lakeBedVertices.size());
	}
	out.reserve(count);
	for (const Lake::Ptr &lake : lakes) {
		out.insert(lake->lakeBedVertices.begin(), lake->lakeBedVertices.end());
	}
	return out;
}

void Lake::carveInto(Mesh &mesh) {
	float minZ = mesh.vertices[outflow].z;
	for (int i : lakeBedVertices) {
		float z = mesh.vertices[i].z;
		if (z < minZ) {
			minZ = z;
		}
	}
	for (int i : lakeBedVertices) {
		mesh.vertices[i].z = minZ;
	}
}

Mesh &Lake::createMesh(const Mesh &in, const MeshTriangleMap &tris, Mesh &out) const {
	std::unordered_set<int> used;
	used.reserve(lakeBedVertices.size());
	for (int i : lakeBedVertices) {
		auto t = tris.vertex(i);
		used.insert(t.first, t.second);
	}
	std::unordered_map<int, int> verts;
	out.triangles.reserve(used.size() * 3);
	out.vertices.reserve(out.triangles.size());
	verts.reserve(out.triangles.size());
	for (int i : used) {
		for (int j = 0; j != 3; ++j) {
			int vert = in.triangles[i + j];
			auto pos = verts.find(vert);
			if (pos == verts.end()) {
				int offset = out.vertices.size();
				verts.emplace(vert, offset);
				out.vertices.push_back(in.vertices[vert]);
				out.normals.push_back(Vector3::unitZ());
				out.triangles.push_back(offset);
			}
			else {
				out.triangles.push_back(pos->second);
			}
		}
	}
	return out;
}

void Lake::mergeLakes(const Mesh &mesh, Lakes &lakes) {
	int count = 0;
	for (const Lake::Ptr &lake : lakes) {
		count += lake->lakeBedVertices.size();
	}
	std::unordered_map<int, int> verts;
	verts.reserve(count);
	for (int i = 0; i != lakes.size(); ++i){
		Lake &lake = *lakes[i];
		for (int vert : lake.lakeBedVertices) {
			auto j = verts.find(vert);
			if (j != verts.end() && j->second != i) {
				for (int k : lakes[j->second]->lakeBedVertices) {
					verts[k] = i;
				}
				lake.flow += lakes[j->second]->flow;
				if (mesh.vertices[lakes[j->second]->outflow].z < mesh.vertices[lake.outflow].z) {
					lake.outflow = lakes[j->second]->outflow;
				}
			}
			else {
				verts[vert] = i;
			}
		}
	}
	for (Lake::Ptr &lake : lakes) {
		lake->lakeBedVertices.clear();
	}
	for (auto i : verts) {
		lakes[i.second]->lakeBedVertices.push_back(i.first);
	}
	Lakes swap(lakes);
	lakes.clear();
	for (const Lake::Ptr &lake : swap) {
		if (!lake->lakeBedVertices.empty()) {
			lakes.push_back(lake);
		}
	}
}