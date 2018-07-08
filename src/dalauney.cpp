#include "dalauney.h"
#include "mesh.h"
#include "rtree.h"

using namespace motu;

//
//  dalauney_triangulation.h
//  World Maker
//
//  Created by Jerome Johnson on 14/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef dalauney_h
#define dalauney_h

#include "vector2.h"
#include "triangle3.h"
#include "mesh.h"

namespace {
	struct TriangleWithCircumcircle {
		Triangle3 triangle;
		Circle circumcircle;

		TriangleWithCircumcircle() {}

		TriangleWithCircumcircle(const Triangle3 &triangle) : triangle(triangle), circumcircle(triangle.toTriangle2().circumcircle()) {}

		template <class Itr>
		void findUnsharedEdges(Itr first, Itr last, std::vector<Spline> &output) {
			Spline myEdges[3];
			triangle.getSplines(myEdges);
			for (int i = 0; i != 3; ++i) {
				bool hasEdge = false;
				for (Itr j = first; j != last; ++j) {
					if (*j == *this) {
						continue;
					}
					if (j->triangle.toTriangle2().shares(myEdges[i].edge())) {
						hasEdge = true;
						break;
					}
				}
				if (!hasEdge) {
					output.push_back(myEdges[i]);
				}
			}
		}

		bool operator==(const TriangleWithCircumcircle &other) const {
			return triangle.toTriangle2() == other.triangle.toTriangle2();
		}
	};

	typedef RTree<TriangleWithCircumcircle, float, 2> Tree;

	Triangle3 CreateBoundingTriangle(const std::vector<Vector3> &points) {
		BoundingRectangle bounds;
		bounds.clear();
		for (const Vector3 &point : points) {
			bounds += point.asVector2();
		}
		bounds.topLeft += Vector2(-1.0f, -1.0f);
		bounds.bottomRight += Vector2(1.0f, 1.0f);
		float width = bounds.width(), height = bounds.height();
		Vector3 apex(bounds.topLeft.x + width * 0.5f, bounds.topLeft.y - width * 0.6667f, 0.0f);
		float spread = height * 0.6667f;
		Vector3 left(bounds.topLeft.x - spread, bounds.bottomRight.y, 0.0f);
		Vector3 right(bounds.bottomRight.x + spread, bounds.bottomRight.y, 0.0f);
		return Triangle3(left, apex, right);
	}

	bool OfSuperTriangle(const Triangle &super, const Triangle &triangle) {
		for (int i = 0; i != 3; ++i) {
			for (int j = 0; j != 3; ++j) {
				if (super.vertices[i] == triangle.vertices[j]) {
					return true;
				}
			}
		}
		return false;
	}

	bool AddBadTriangles(const Vector2 &point, std::vector<TriangleWithCircumcircle> &badTriangles, const Tree &rTree/*, typename RTree::Values &buf*/) {
		rTree.Search(&point.x, &point.x, [&badTriangles, &point](const TriangleWithCircumcircle &tri) {
			if (tri.triangle.toTriangle2().hasVertex(point)) {
				return false;
			}
			if (tri.circumcircle.contains(point)) {
				badTriangles.push_back(tri);
			}
			return true;
		});
		return true;
	}
}

Mesh &motu::createDalauneyMesh(const std::vector<Vector3> &vertices, Mesh &out) {
	int expectedTriangles = static_cast<int>(vertices.size() - 2);
	Tree rTree;
	TriangleWithCircumcircle super(CreateBoundingTriangle(vertices));
	BoundingRectangle rect = super.circumcircle.boundingRectangle();
	rTree.Insert(&rect.topLeft.x, &rect.bottomRight.x, super);
	std::vector<TriangleWithCircumcircle> badTriangles;
	std::vector<Spline> poly;
	for (const Vector3 &point : vertices) {
		if (!AddBadTriangles(point.asVector2(), badTriangles, rTree)) {
			continue;
		}
		for (auto i = badTriangles.begin(); i != badTriangles.end(); ++i) {
			i->findUnsharedEdges(badTriangles.begin(), badTriangles.end(), poly);
		}
		for (auto bad : badTriangles) {
			rect = bad.circumcircle.boundingRectangle();
			rTree.Remove(&rect.topLeft.x, &rect.bottomRight.x, bad);
		}
		for (auto edge : poly) {
			TriangleWithCircumcircle tri(Triangle3(edge.endA, edge.endB, point));
			if (tri.triangle.normal().z <= 0.0f) {
				tri.triangle.flipRotation();
			}
			rect = tri.circumcircle.boundingRectangle();
			rTree.Insert(&rect.topLeft.x, &rect.bottomRight.x, tri);
		}
		poly.clear();
		badTriangles.clear();
	}
	std::vector<Triangle3> t3s;
	t3s.reserve(rTree.Count());
	Triangle supt2 = super.triangle.toTriangle2();
	Tree::Iterator itr;
	for (rTree.GetFirst(itr); itr.IsNotNull(); rTree.GetNext(itr)) {
		if (!OfSuperTriangle(supt2, (*itr).triangle.toTriangle2())) {
			t3s.push_back((*itr).triangle);
		}
	}
	out.load(t3s);
	return out;
}

#endif