//
//  mesh.cpp
//  World Maker
//
//  Created by Jerome Johnson on 20/11/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#include "mesh.h"
#include <queue>
#include "plane.h"
#include <vector>
#include "triangle3.h"
#include <algorithm>
#include "brtree.h"
#include "triangle.h"
#include "bounding_box.h"
#include <unordered_map>
#include <unordered_set>
#include "util.h"
#include "mesh_edge_map.h"
#include "mesh_triangle_map.h"
#include "matrix4.h"

using namespace motu;

namespace{

	typedef BRTree<BoundingRectangle, size_t> RefRtree;

	inline void interpolated(const Vector3 &endA, const Vector3 &endB, float hitTime, Vector3 &out) {
		out = endA + ((endB - endA) * hitTime);
	}

	inline void emplacePreserveRotation(const Vector3 normal, std::vector<Triangle3> &out, const Vector3 &a, const Vector3 &b, const Vector3 &c) {
		if ((b - a).cross(c - b).dot(normal) < 0.0f) {
			out.emplace_back(c, b, a);
		}
		else {
			out.emplace_back(a, b, c);
		}
	}

	template<class Tri, class InOut>
	bool scanBounds(const Tri &tri, float y, InOut &in, InOut &out){
        in.vertex.x = std::numeric_limits<float>::max();
        out.vertex.x = std::numeric_limits<float>::min();
        int found = 0;
		for (int i = 0; i != 3; ++i) {
			const Vector3 &a = tri.vertices[i], &b = tri.vertices[(i + 1) % 3];
			Edge edge(a.x, a.y, b.x, b.y);
            Vector2 direction(edge.direction());
            float x = !almostZero(direction.y) ? (((y * direction.x) - (edge.endA.y * direction.x))/ direction.y) + edge.endA.x : edge.endA.x;
            if (Edge::between(edge.endA.x, edge.endB.x, x)){
				float denom = edge.endB.x - edge.endA.x;
				float hitAt = !almostZero(denom) ? (x - edge.endA.x) / denom : 0.0f;
				InOut io;
				io.interpolate(tri, i, (i + 1) % 3, hitAt);
                if (x < in.vertex.x){
					in = io;
                }
                if (x > out.vertex.x){
                    out = io;
                }
                ++found;
            }
            if (found == 2){
                return true;
            }
        }
        return false;
    }
    
	template <class Raster, class Tri>
    void fillTriangle(Raster &grid, const Tri &tri){
        float minY = std::numeric_limits<float>::max(), maxY = std::numeric_limits<float>::min();
		for (int i = 0; i != 3; ++i) {
            float y = tri.vertices[i].y;
            if (y < minY){
                minY = y;
            }
            if (y > maxY){
                maxY = y;
            }
        }
        int h = grid.height(), w = grid.width();
        float stepSize = 1.0f / static_cast<float>(h);
        for (int y = static_cast<int>(minY * h), yend = std::min(static_cast<int>(maxY * static_cast<float>(h)), h - 1); y <= yend; ++y){
            if (y < 0){
                continue;
            }
            float fy = static_cast<float>(y) * stepSize;
            Tri::InOut in, out;
            if (scanBounds(tri, fy, in, out)){
                int x = static_cast<int>(in.vertex.x * static_cast<float>(w)), xend = static_cast<int>(out.vertex.x * static_cast<float>(w));
                float denom = static_cast<float>(xend - x);
                if (denom == 0.0f){
					if (x >= 0 && x < w) {
						in.assignTo(grid(x, y));
					}
                    continue;
                }
                Tri::InOut step(in, out, denom);
                while (x < 0){
					in += step;
                    ++x;
                }
                xend = std::min(xend, w - 1);
                while (x <= xend){
                    in.assignTo(grid(x, y));
					in += step;
                    ++x;
                }
            }
        }
    }

	struct TriWithNormals : public Triangle3WithNormals {
		struct InOut : public Mesh::VertexAndNormal {
			InOut() {}

			InOut(const InOut &in, const InOut &out, float denom) {
				vertex = (out.vertex - in.vertex) / denom;
				normal = (out.normal - in.normal) / denom;
			}

			InOut &operator+=(const InOut &add) {
				vertex += add.vertex;
				normal += add.normal;
				return *this;
			}

			void interpolate(const Triangle3WithNormals &tri, int endA, int endB, float at) {
				interpolated(tri.vertices[endA], tri.vertices[endB], at, vertex);
				interpolated(tri.normals[endA], tri.normals[endB], at, normal);
			}

			void assignTo(Mesh::VertexAndNormal &van) const{
				van = *this;
			}
		};
	};

	struct Normals : public Triangle3WithNormals {
		struct InOut : public Mesh::VertexAndNormal {
			InOut() {}

			InOut(const InOut &in, const InOut &out, float denom) {
				vertex = (out.vertex - in.vertex) / denom;
				normal = (out.normal - in.normal) / denom;
			}

			InOut &operator+=(const InOut &add) {
				vertex += add.vertex;
				normal += add.normal;
				return *this;
			}

			void interpolate(const Triangle3WithNormals &tri, int endA, int endB, float at) {
				interpolated(tri.vertices[endA], tri.vertices[endB], at, vertex);
				interpolated(tri.normals[endA], tri.normals[endB], at, normal);
			}

			void assignTo(Vector3 &norm) const {
				norm = normal;
			}
		};
	};


	struct Tri : public Triangle3 {
		struct InOut {
			Vector3 vertex;

			InOut() {}

			InOut(const InOut &in, const InOut &out, float denom) {
				vertex = (out.vertex - in.vertex) / denom;
			}

			InOut &operator+=(const InOut &add) {
				vertex += add.vertex;
				return *this;
			}

			void interpolate(const Triangle3 &tri, int endA, int endB, float at) {
				interpolated(tri.vertices[endA], tri.vertices[endB], at, vertex);
			}

			void assignTo(Vector3 &v) const {
				v = vertex;
			}
		};
	};

	struct Height : public Triangle3{ 
		struct InOut {
			Vector3 vertex;

			InOut() {}

			InOut(const InOut &in, const InOut &out, float denom) {
				vertex = (out.vertex - in.vertex) / denom;
			}

			InOut &operator+=(const InOut &add) {
				vertex += add.vertex;
				return *this;
			}

			void interpolate(const Triangle3 &tri, int endA, int endB, float at) {
				interpolated(tri.vertices[endA], tri.vertices[endB], at, vertex);
			}

			void assignTo(float &v) const {
				v = vertex.z;
			}
		};
	};

	void splitTriangles(const Plane &divider, std::vector<Triangle3WithNormals> &in, std::vector<Triangle3WithNormals> &out) {
		//size_t i = out.size();
		for (const Triangle3WithNormals &tri : in) {
			tri.slice(divider, out);
		}
	}

	inline Triangle3 getTriangle(const Mesh &mesh, int offset) {
		return Triangle3(
			mesh.vertices[mesh.triangles[offset]],
			mesh.vertices[mesh.triangles[offset + 1]],
			mesh.vertices[mesh.triangles[offset + 2]]
		);
	}

	inline Triangle3WithNormals getTriangleWithNormals (const Mesh &mesh, int offset) {
		int a = mesh.triangles[offset], b = mesh.triangles[offset + 1], c = mesh.triangles[offset + 2];
		return Triangle3WithNormals(
			mesh.vertices[a],
			mesh.vertices[b],
			mesh.vertices[c],
			mesh.normals[a],
			mesh.normals[b],
			mesh.normals[c]
		);
	}

	struct Flip {
		Vector3 a, b, normal;
		bool complete;

		Flip(const Vector3 &vec, const Vector3 &normal) : a(vec), normal(normal), complete(false) {
		}
	};

	typedef std::pair<int, int> OffsetPair;

	struct OffsetPairHasher {
		size_t operator()(const OffsetPair &pair) const {
			return (pair.first * HASH_PRIME_A) ^ (pair.second * HASH_PRIME_B);
		}
	};

	typedef std::unordered_map<OffsetPair, Flip, OffsetPairHasher> FlipMap;
	typedef std::vector<Triangle3> Triangles;

	void tesselateTriangle(const Mesh &mesh, int offset, FlipMap &flips, Triangles &out) {
		Triangle3 tri(getTriangle(mesh, offset));
		Vector3 norm(tri.normal());
		Vector3 centre(tri.baricentre());
		Spline splines[3];
		tri.getSplines(splines);
		for (int j = 0; j != 3; ++j) {
			if (splines[j].direction().squareMagnitude() < FLT_EPSILON) {
				out.push_back(tri);
				return;
			}
		}
		for (int j = 0; j != 3; ++j) {
			OffsetPair spline(mesh.triangles[offset + j], mesh.triangles[offset + ((j + 1) % 3)]);
			bool swapped = spline.second < spline.first;
			if (swapped) {
				std::swap(spline.first, spline.second);
			}
			auto k = flips.find(spline);
			if (k != flips.end()) {
				k->second.b = centre;
				k->second.complete = true;
			}
			else {
				flips.emplace_hint(k, spline, Flip(centre, norm));
			}
		}
	}

	void addEdge(Mesh::PerimeterSet &perm, const Mesh &mesh, int endA, int endB) {
		endA = mesh.triangles[endA];
		endB = mesh.triangles[endB];
		if (endB < endA) {
			std::swap(endA, endB);
		}
		Mesh::PerimeterSet::OffsetPair op(endA, endB);
		auto i = perm.edgeSet.find(op);
		if (i == perm.edgeSet.end()) {
			perm.edgeSet.emplace_hint(i, op);
		}
		else {
			perm.edgeSet.erase(i);
		}
	}

	template <class TriType, class GridType>
	void rasterize(const Mesh &mesh, TriType &tri, GridType &raster) {
		for (int i = 0; i + 2 < mesh.triangles.size();) {
			for (int j = 0; j != 3; ++j, ++i) {
				size_t offset = mesh.triangles[i];
				tri.vertices[j] = mesh.vertices[offset];
			}
			fillTriangle(raster, tri);
		}
	}

	template <class TriType, class GridType>
	void rasterizeNormals (const Mesh &mesh, TriType &tri, GridType &raster) {
		for (int i = 0; i + 2 < mesh.triangles.size();) {
			for (int j = 0; j != 3; ++j, ++i) {
				int offset = mesh.triangles[i];
				tri.vertices[j] = mesh.vertices[offset];
				tri.normals[j] = mesh.normals[offset];
			}
			fillTriangle(raster, tri);
		}
	}
	//right, bottom, skip, left, top, skip
	template<class TriangleSplitter>
	Mesh &performSlice(const Mesh src, const BoundingBox &bounds, Mesh &out, TriangleSplitter splitter) {
		std::vector<Triangle3WithNormals> inside, selection, split;
		inside.reserve(src.triangles.size() / 3);
		for (int i = 2; i < src.triangles.size(); i += 3) {
			Triangle3WithNormals tri(getTriangleWithNormals(src, i - 2));
			if (bounds.intersects(tri)) {
				if (bounds.contains(tri)) {
					inside.push_back(tri);
				}
				else {
					split.push_back(tri);
				}
			}
		}
		Plane planes[6];
		bounds.getPlanes(planes);
		for (int i = 0; i != 6; ++i) {
			selection.clear();
			for (const Triangle3WithNormals &tri : split) {
				if (tri.intersects(planes[i])) {
					selection.push_back(tri);
				}
			}
			uint8_t side;
			switch (i) {
			case 0:
				side = Mesh::RIGHT;
				break;
			case 1:
				side = Mesh::BOTTOM;
				break;
			case 3:
				side = Mesh::LEFT;
				break;
			case 4:
				side = Mesh::TOP;
				break;
			default:
				side = 0;
			}
			splitter(side, planes[i], selection, split);
		}
		for (const Triangle3WithNormals &tri : split) {
			if (bounds.contains(tri)) {
				inside.push_back(tri);
			}
		}
		out.load(inside);
		return out;
	}

	struct MeshClamper {
		Mesh clamp;
		std::vector<Triangle3WithNormals> buffer;
		std::vector<int> clampLine;
		std::vector<std::pair<int, int>> onLine;
		uint8_t sides;

		template <class GetDim>
		void performClamp(GetDim getDim) {
			if (clampLine.size() < 2) {
				return;
			}
			const Vector3 *lastVert = clamp.vertices.data() + clampLine.back(), *lastNormal = clamp.normals.data() + clampLine.back();
			clampLine.pop_back();
			const Vector3 *nextVert = clamp.vertices.data() + clampLine.back(), *nextNormal = clamp.normals.data() + clampLine.back();
			clampLine.pop_back();
			float invDist = 1.0f / (getDim(*nextVert) - getDim(*lastVert));
			int i = 0;
			while (i != onLine.size()) {
				const std::pair<int, int> &vn = onLine[i];
				Vector3 &vert = buffer[vn.first].vertices[vn.second];
				if (getDim(*nextVert) < getDim(vert)) {
					if (clampLine.empty()) {
						break;
					}
					lastVert = nextVert;
					lastNormal = nextNormal;
					nextVert = clamp.vertices.data() + clampLine.back();
					nextNormal = clamp.normals.data() + clampLine.back();
					invDist = 1.0f / (getDim(*nextVert) - getDim(*lastVert));
					clampLine.pop_back();
				}
				float hit = (getDim(vert) - getDim(*lastVert)) * invDist;
				if (hit >= 0.0f && hit <= 1.0f) {
					vert.z = ((1.0f - hit) * lastVert->z) + (hit * nextVert->z);
					buffer[vn.first].normals[vn.second] = (*lastNormal * (1.0f - hit)) + (*nextNormal * hit);
					buffer[vn.first].normals[vn.second].normalize();
				}
				++i;
			}
			while (i != onLine.size()) {
				const std::pair<int, int> &vn = onLine[i++];
				buffer[vn.first].vertices[vn.second].z = lastVert->z;
				buffer[vn.first].normals[vn.second] = *lastNormal;
			}
		}

		template<class GetDimension, class GetPerp>
		void createZLookup(float planeDim, GetDimension getDim, GetPerp getPerp) {
			for (int i = 0; i != clamp.vertices.size(); ++i) {
				if (motu::almostEqual(planeDim, getDim(clamp.vertices[i]))) {
					clampLine.push_back(i);
				}
			}
			std::sort(clampLine.begin(), clampLine.end(), [this, getPerp](int a, int b) {
				return getPerp(clamp.vertices[a]) > getPerp(clamp.vertices[b]);
			});
		}

		template<class GetDimension, class GetPerp>
		void glue(const Plane &divider, std::vector<Triangle3WithNormals> &in, std::vector<Triangle3WithNormals> &out, GetDimension getDim, GetPerp getPerp) {
			buffer.clear();
			splitTriangles(divider, in, buffer);
			if (buffer.empty()) {
				return;
			}
			clampLine.clear();
			onLine.clear();
			float planeDim = getDim(divider.point);
			for (int i = 0; i != buffer.size(); ++i) {
				for (int j = 0; j != 3; ++j) {
					if (almostEqual(planeDim, getDim(buffer[i].vertices[j]))) {
						onLine.emplace_back(i, j);
					}
				}
			}
			std::sort(onLine.begin(), onLine.end(), [this, getPerp](const std::pair<int, int> &a, const std::pair<int, int> &b) {
				return getPerp(buffer[a.first].vertices[a.second]) < getPerp(buffer[b.first].vertices[b.second]);
			});
			createZLookup(planeDim, getDim, getPerp);
			performClamp(getPerp);
			std::copy(buffer.begin(), buffer.end(), std::back_inserter(out));
		}
	};
}

BoundingBox &Mesh::getMaxSquare(BoundingBox &out) const{
	PerimeterSet perimeter;
	getPerimeterSet(perimeter);
	BoundingRectangle bounds;
	bounds.clear();
	for (int i : perimeter) {
		Vector2 vec2 = vertices[i].asVector2();
		bounds.add(vec2);
	}
	motu::Edge a(bounds.topLeft, bounds.bottomRight), b(bounds.topLeft.x, bounds.bottomRight.y, bounds.bottomRight.x, bounds.topLeft.y);
	bounds.clear();
	BoundingRectangle bBounds;
	bBounds.clear();
	for (auto i = perimeter.edgeSet.begin(); i != perimeter.edgeSet.end(); ++i) {
		motu::Edge edge(vertices[i->first].asVector2(), vertices[i->second].asVector2());
		Vector2 intersection;
		if (edge.intersects(a, intersection)) {
			bounds.add(intersection);
		}
		else if (edge.intersects(b, intersection)) {
			bBounds.add(intersection);
		}
	}
	out.upper.x = std::max(bounds.topLeft.x, bBounds.topLeft.x);
	out.upper.y = std::max(bounds.topLeft.y, bBounds.topLeft.y);
	out.upper.z = -1.0f;
	out.lower.x = std::min(bounds.bottomRight.x, bBounds.bottomRight.x);
	out.lower.y = std::min(bounds.bottomRight.y, bBounds.bottomRight.y);
	out.lower.z = 1.0f;
	float width = out.lower.x - out.upper.x, height = out.lower.y - out.upper.y;
	if (width < height) {
		float inset = (height - width) * 0.5f;
		out.upper.y += inset;
		out.lower.y -= inset;
	}
	else {
		float inset = (width - height) * 0.5f;
		out.upper.x += inset;
		out.lower.x -= inset;
	}
	return out;
}

Mesh &Mesh::tesselate() {
	FlipMap flips;
	flips.reserve(triangles.size());
	std::vector<Triangle3> out;
	out.reserve(triangles.size() * 3);
	for (int i = 2; i < triangles.size(); i += 3) {
		tesselateTriangle(*this, i - 2, flips, out);
	}
	for (auto i = flips.begin(); i != flips.end(); ++i) {
		if (i->second.complete) {
			emplacePreserveRotation(i->second.normal, out, i->second.a, i->second.b, vertices[i->first.first]);
			emplacePreserveRotation(i->second.normal, out, i->second.a, i->second.b, vertices[i->first.second]);
		}
		else{
			emplacePreserveRotation(i->second.normal, out, vertices[i->first.first], i->second.a, vertices[i->first.second]);
		}
	}
	clear();
	load(out);
	return *this;
}

void Mesh::load(std::vector<Triangle3> &tris) {
	std::unordered_map<Vector3, int, Hasher<Vector3>> vertexMap;
	int numVerts = static_cast<int>(tris.size()) + 2;
	vertexMap.reserve(numVerts);
	triangles.reserve(tris.size());
	vertices.reserve(numVerts);
	for (auto i = tris.begin(); i != tris.end(); ++i) {
		for (int j = 0; j != 3; ++j) {
			const Vector3 &vert = i->vertices[j];
			auto k = vertexMap.find(vert);
			if (k != vertexMap.end()) {
				triangles.push_back(k->second);
			}
			else {
				int pos = static_cast<int>(vertices.size());
				vertices.push_back(vert);
				triangles.push_back(pos);
				vertexMap.emplace(vert, pos);
			}
		}
	}
}

void Mesh::load(std::vector<Triangle3WithNormals> &tris) {
	std::unordered_map<Vector3, int, Hasher<Vector3>> vertexMap;
	int numVerts = static_cast<int>(tris.size()) + 2;
	vertexMap.reserve(numVerts);
	triangles.reserve(tris.size());
	vertices.reserve(numVerts);
	normals.reserve(numVerts);
	for (auto i = tris.begin(); i != tris.end(); ++i) {
		for (int j = 0; j != 3; ++j) {
			const Vector3 &vert = i->vertices[j];
			auto k = vertexMap.find(vert);
			if (k != vertexMap.end()) {
				triangles.push_back(k->second);
			}
			else {
				int pos = static_cast<int>(vertices.size());
				vertices.push_back(vert);
				normals.push_back(i->normals[j]);
				triangles.push_back(pos);
				vertexMap.emplace(vert, pos);
			}
		}
	}
}

void Mesh::calculateNormals() {
	normals.reserve(vertices.size());
	MeshTriangleMap triMap(*this);
	for (int i = 0; i != vertices.size(); ++i) {
		Vector3 normal = Vector3::zero();
		auto tris = triMap.vertex(i);
		while (tris.first != tris.second) {
			int offset = *(tris.first++);
			normal += Triangle3(vertices[triangles[offset]], vertices[triangles[offset + 1]], vertices[triangles[offset + 2]]).normal();
		}
		normal.normalize();
		normals.push_back(normal);
	}
}

void Mesh::smooth() {
	MeshEdgeMap edges(*this);
	PerimeterSet perimeter;
	getPerimeterSet(perimeter);
	Vector3 *moved = new Vector3[vertices.size()];
	for (int i = 0; i != vertices.size(); ++i) {
		if (perimeter.find(i) != perimeter.end()) {
			moved[i] = vertices[i];
			continue;
		}
		Vector3 total(vertices[i]);
		std::pair<const int*, const int*> offsets(edges.vertex(i));
		float count = static_cast<float>((offsets.second - offsets.first) + 1);
		while (offsets.first != offsets.second) {
			total += vertices[*(offsets.first++)];
		}
		moved[i] = total / count;
	}
	std::copy(moved, moved + vertices.size(), vertices.data());
	delete[] moved;
}

void Mesh::rasterize(Grid<VertexAndNormal> &raster) const{
	TriWithNormals tri;
	for (int i = 0; i + 2 < triangles.size();) {
		for (int j = 0; j != 3; ++j, ++i) {
			int offset = triangles[i];
			tri.vertices[j] = vertices[offset];
			tri.normals[j] = normals[offset];
		}
		fillTriangle(raster, tri);
	}
}

void Mesh::rasterizeNormalsOnly(Grid<Vector3> &raster) const {
	rasterizeNormals(*this, ::Normals(), raster);
}

void Mesh::rasterize(Grid<Vector3> &raster) const {
	::rasterize(*this, Tri(), raster);
}

void Mesh::rasterize(Grid<float> &raster) const {
	::rasterize(*this, Height(), raster);
}

Mesh::Edges &Mesh::edges(Edges &edges) const {
	for (int i = 2; i < triangles.size(); i += 3) {
		edges.emplace(triangles[i - 2], triangles[i - 1]);
		edges.emplace(triangles[i - 1], triangles[i]);
		edges.emplace(triangles[i], triangles[i - 2]);
	}
	return edges;
}

Mesh &Mesh::slice(const BoundingBox &bounds, Mesh &out) const {
	performSlice(*this, bounds, out, [](uint8_t side, const Plane &divider, std::vector<Triangle3WithNormals> &in, std::vector<Triangle3WithNormals> &out) {
		splitTriangles(divider, in, out);
	});
	return out;
}

Mesh &Mesh::slice(const BoundingBox &bounds, uint8_t clampDown, const Mesh &clampMesh, Mesh &out) const{
	MeshClamper clamper;
	clamper.sides = clampDown;
	clampMesh.slice(bounds, clamper.clamp);
	performSlice(*this, bounds, out, [&clamper](uint8_t side,  Plane &divider, std::vector<Triangle3WithNormals> &in, std::vector<Triangle3WithNormals> &out) {
		if (!(clamper.sides & side)) {
			splitTriangles(divider, in, out);
		}
		else if (divider.normal.x != 0.0f) {
			clamper.glue(divider, in, out, [](const Vector3 &vec) {
				return vec.x;
			}, [](const Vector3 &vec) {
				return vec.y;
			});
		}
		else {
			clamper.glue(divider, in, out, [](const Vector3 &vec) {
				return vec.y;
			}, [](const Vector3 &vec) {
				return vec.x;
			});
		}
	});
	return out;
}

Mesh &Mesh::transform(const Matrix4 &m4) {
	for (Vector3 &v3 : vertices) {
		v3 = (m4 * Matrix4::asPoint(v3)).asVector3();
	}
	return *this;
}

Mesh::PerimeterSet &Mesh::getPerimeterSet(Mesh::PerimeterSet &perm) const {
	perm.edgeSet.reserve(triangles.size());
	for (int i = 2; i < triangles.size(); i += 3) {
		addEdge(perm, *this, i - 2, i - 1);
		addEdge(perm, *this, i - 1, i);
		addEdge(perm, *this, i, i - 2);
	}
	perm.reserve(perm.edgeSet.size());
	for (const OffsetPair &op : perm.edgeSet) {
		perm.insert(op.first);
		perm.insert(op.second);
	}
	return perm;
}
