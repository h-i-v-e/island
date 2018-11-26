#include "triangle3.h"
#include <tuple>

using namespace motu;

namespace {
	void emplaceWithRotation(const Vector3 &normal, const Vector3 &a, const Vector3 &b, const Vector3 &c, std::vector<Triangle3> &out) {
		if ((b - a).cross(c - b).dot(normal) < 0.0f) {
			out.emplace_back(c, b, a);
		}
		else {
			out.emplace_back(a, b, c);
		}
	}

	void emplaceWithRotation(const Vector3 &normal,
		const Vector3 &a, const Vector3 &b, const Vector3 &c,
		const Vector3 &na, const Vector3 &nb, const Vector3 &nc,
		std::vector<Triangle3WithNormals> &out) {
		if ((b - a).cross(c - b).dot(normal) < 0.0f) {
			out.emplace_back(c, b, a, nc, nb, na);
		}
		else {
			out.emplace_back(a, b, c, na, nb, nc);
		}
	}

	void emplaceWithRotation(const Vector3 &normal,
		const Vector3 &a, const Vector3 &b, const Vector3 &c,
		const Vector3 &na, const Vector3 &nb, const Vector3 &nc,
		const Vector2 &ua, const Vector2 &ub, const Vector2 &uc,
		std::vector<Triangle3WithNormalsAndUV> &out) {
		if ((b - a).cross(c - b).dot(normal) < 0.0f) {
			out.emplace_back(c, b, a, nc, nb, na, uc, ub, ua);
		}
		else {
			out.emplace_back(a, b, c, na, nb, nc, ua, ub, uc);
		}
	}
}

bool Triangle3::intersection(const Spline &spline, Vector3 &intersection) const {
	Vector3 vertex0 = vertices[0];
	Vector3 vertex1 = vertices[1];
	Vector3 vertex2 = vertices[2];
	Vector3 edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	Vector3 rayVector = spline.direction(), rayOrigin = spline.endA;
	h = rayVector.cross(edge2);
	a = edge1.dot(h);
	if (a > -FLT_EPSILON && a < FLT_EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0 / a;
	s = rayOrigin - vertex0;
	u = f * (s.dot(h));
	if (u < 0.0 || u > 1.0)
		return false;
	q = s.cross(edge1);
	v = f * rayVector.dot(q);
	if (v < 0.0 || u + v > 1.0)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * edge2.dot(q);
	if (t > FLT_EPSILON) // ray intersection
	{
		intersection = (rayOrigin + rayVector * t);
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}

std::vector<Triangle3> &Triangle3::slice(const Plane &plane, std::vector<Triangle3> &out) const {
	Spline splines[3];
	getSplines(splines);
	std::pair<Vector3, const Spline*> intersection[3];
	Vector3 normal(this->normal());
	size_t offset = 0;
	const Vector3 *a, *b, *c, *d, *e;
	for (int i = 0; i != 3; ++i) {
		if (splines[i].intersects(plane, intersection[offset].first)) {
			intersection[offset++].second = splines + i;
		}
	}
	switch (offset) {
	case 2:
		if (intersection[0].second->endA == intersection[1].second->endA) {
			a = &intersection[0].second->endA;
			b = &intersection[0].first;
			c = &intersection[0].second->endB;
			d = &intersection[1].second->endB;
			e = &intersection[1].first;
		}
		else if (intersection[0].second->endA == intersection[1].second->endB) {
			a = &intersection[0].second->endA;
			b = &intersection[0].first;
			c = &intersection[0].second->endB;
			d = &intersection[1].second->endA;
			e = &intersection[1].first;
		}
		else if (intersection[0].second->endB == intersection[1].second->endA) {
			a = &intersection[1].second->endA;
			b = &intersection[1].first;
			c = &intersection[1].second->endB;
			d = &intersection[0].second->endA;
			e = &intersection[0].first;
		}
		else/* if (intersection[0].second->endB == intersection[1].second->endB) */{
			a = &intersection[0].second->endB;
			b = &intersection[0].first;
			c = &intersection[0].second->endA;
			d = &intersection[1].second->endA;
			e = &intersection[1].first;
		}
		emplaceWithRotation(normal, *a, *b, *e, out);
		emplaceWithRotation(normal, *b, *c, *d, out);
		emplaceWithRotation(normal, *e, *b, *d, out);
		break;
	case 3:
		if (intersection[0].first == intersection[1].first) {
			emplaceWithRotation(normal, intersection[0].second->endA, intersection[2].first, intersection[0].second->endB, out);;
			emplaceWithRotation(normal, intersection[1].second->endA, intersection[2].first, intersection[1].second->endB, out);
		}
		else if (intersection[0].first == intersection[2].first) {
			emplaceWithRotation(normal, intersection[0].second->endA, intersection[1].first, intersection[0].second->endB, out);
			emplaceWithRotation(normal, intersection[2].second->endA, intersection[1].first, intersection[2].second->endB, out);
		}
		else {
			emplaceWithRotation(normal, intersection[1].second->endA, intersection[0].first, intersection[1].second->endB, out);
			emplaceWithRotation(normal, intersection[2].second->endA, intersection[0].first, intersection[2].second->endB, out);
		}
	}
	return out;
}

std::vector<Triangle3WithNormals> &Triangle3WithNormals::slice(const Plane &plane, std::vector<Triangle3WithNormals> &out) const{
	SplineWithNormals splines[3];
	getSplines(splines);
	std::pair<std::pair<Vector3, Vector3>, const SplineWithNormals*> intersection[3];
	Vector3 normal(this->normal());
	size_t offset = 0;
	const Vector3 *a, *b, *c, *d, *e, *na, *nb, *nc, *nd, *ne;
	for (int i = 0; i != 3; ++i) {
		if (splines[i].intersects(plane, intersection[offset].first.first, intersection[offset].first.second)) {
			intersection[offset++].second = splines + i;
		}
	}
	switch (offset) {
	case 2:
		if (intersection[0].second->endA == intersection[1].second->endA) {
			a = &intersection[0].second->endA;
			na = &intersection[0].second->normalA;
			b = &intersection[0].first.first;
			nb = &intersection[0].first.second;
			c = &intersection[0].second->endB;
			nc = &intersection[0].second->normalB;
			d = &intersection[1].second->endB;
			nd = &intersection[1].second->normalB;
			e = &intersection[1].first.first;
			ne = &intersection[1].first.second;
		}
		else if (intersection[0].second->endA == intersection[1].second->endB) {
			a = &intersection[0].second->endA;
			na = &intersection[0].second->normalA;
			b = &intersection[0].first.first;
			nb = &intersection[0].first.second;
			c = &intersection[0].second->endB;
			nc = &intersection[0].second->normalB;
			d = &intersection[1].second->endA;
			nd = &intersection[1].second->normalA;
			e = &intersection[1].first.first;
			ne = &intersection[1].first.second;
		}
		else if (intersection[0].second->endB == intersection[1].second->endA) {
			a = &intersection[1].second->endA;
			na = &intersection[1].second->normalA;
			b = &intersection[1].first.first;
			nb = &intersection[1].first.second;
			c = &intersection[1].second->endB;
			nc = &intersection[1].second->normalB;
			d = &intersection[0].second->endA;
			nd = &intersection[0].second->normalA;
			e = &intersection[0].first.first;
			ne = &intersection[0].first.second;
		}
		else/* if (intersection[0].second->endB == intersection[1].second->endB) */{
			a = &intersection[0].second->endB;
			na = &intersection[0].second->normalB;
			b = &intersection[0].first.first;
			nb = &intersection[0].first.second;
			c = &intersection[0].second->endA;
			nc = &intersection[0].second->normalA;
			d = &intersection[1].second->endA;
			nd = &intersection[1].second->normalA;
			e = &intersection[1].first.first;
			ne = &intersection[1].first.second;
		}
		emplaceWithRotation(normal, *a, *b, *e, *na, *nb, *ne, out);
		emplaceWithRotation(normal, *b, *c, *d, *nb, *nc, *nd, out);
		emplaceWithRotation(normal, *e, *b, *d, *ne, *nb, *nd, out);
		break;
	case 3:
		if (intersection[0].first == intersection[1].first) {
			emplaceWithRotation(normal, intersection[0].second->endA, intersection[2].first.first, intersection[0].second->endB,
				intersection[0].second->normalA, intersection[2].first.second, intersection[0].second->normalB, out);
			emplaceWithRotation(normal, intersection[1].second->endA, intersection[2].first.first, intersection[1].second->endB,
				intersection[1].second->normalA, intersection[2].first.second, intersection[1].second->normalB, out);
		}
		else if (intersection[0].first == intersection[2].first) {
			emplaceWithRotation(normal, intersection[0].second->endA, intersection[1].first.first, intersection[0].second->endB,
				intersection[0].second->normalA, intersection[1].first.second, intersection[0].second->normalB, out);
			emplaceWithRotation(normal, intersection[2].second->endA, intersection[1].first.first, intersection[2].second->endB,
				intersection[2].second->normalA, intersection[1].first.second, intersection[2].second->normalB, out);
		}
		else {
			emplaceWithRotation(normal, intersection[1].second->endA, intersection[0].first.first, intersection[1].second->endB,
				intersection[1].second->normalA, intersection[0].first.second, intersection[1].second->normalB, out);
			emplaceWithRotation(normal, intersection[2].second->endA, intersection[0].first.first, intersection[2].second->endB,
				intersection[2].second->normalA, intersection[0].first.second, intersection[2].second->normalB, out);
		}
	}
	return out;
}

std::vector<Triangle3WithNormalsAndUV> &Triangle3WithNormalsAndUV::slice(const Plane &plane, std::vector<Triangle3WithNormalsAndUV> &out) const {
	SplineWithNormalsAndUV splines[3];
	getSplines(splines);
	typedef std::tuple<Vector3, Vector3, Vector2> Intr;
	std::pair<Intr, const SplineWithNormalsAndUV*> intersection[3];
	Vector3 normal(this->normal());
	size_t offset = 0;
	const Vector3 *a, *b, *c, *d, *e, *na, *nb, *nc, *nd, *ne;
	const Vector2 *ua, *ub, *uc, *ud, *ue;
	for (int i = 0; i != 3; ++i) {
		Intr &intr = intersection[offset].first;
		if (splines[i].intersects(plane, std::get<0>(intr), std::get<1>(intr), std::get<2>(intr))) {
			intersection[offset++].second = splines + i;
		}
	}
	switch (offset) {
	case 2:
		if (intersection[0].second->endA == intersection[1].second->endA) {
			a = &intersection[0].second->endA;
			na = &intersection[0].second->normalA;
			ua = &intersection[0].second->uvA;
			b = &std::get<0>(intersection[0].first);
			nb = &std::get<1>(intersection[0].first);
			ub = &std::get<2>(intersection[0].first);
			c = &intersection[0].second->endB;
			nc = &intersection[0].second->normalB;
			uc = &intersection[0].second->uvB;
			d = &intersection[1].second->endB;
			nd = &intersection[1].second->normalB;
			ud = &intersection[1].second->uvB;
			e = &std::get<0>(intersection[1].first);
			ne = &std::get<1>(intersection[1].first);
			ue = &std::get<2>(intersection[1].first);
		}
		else if (intersection[0].second->endA == intersection[1].second->endB) {
			a = &intersection[0].second->endA;
			na = &intersection[0].second->normalA;
			ua = &intersection[0].second->uvA;
			b = &std::get<0>(intersection[0].first);
			nb = &std::get<1>(intersection[0].first);
			ub = &std::get<2>(intersection[0].first);
			c = &intersection[0].second->endB;
			nc = &intersection[0].second->normalB;
			uc = &intersection[0].second->uvB;
			d = &intersection[1].second->endA;
			nd = &intersection[1].second->normalA;
			ud = &intersection[1].second->uvA;
			e = &std::get<0>(intersection[1].first);
			ne = &std::get<1>(intersection[1].first);
			ue = &std::get<2>(intersection[1].first);
		}
		else if (intersection[0].second->endB == intersection[1].second->endA) {
			a = &intersection[1].second->endA;
			na = &intersection[1].second->normalA;
			ua = &intersection[1].second->uvA;
			b = &std::get<0>(intersection[1].first);
			nb = &std::get<1>(intersection[1].first);
			ub = &std::get<2>(intersection[1].first);
			c = &intersection[1].second->endB;
			nc = &intersection[1].second->normalB;
			uc = &intersection[1].second->uvB;
			d = &intersection[0].second->endA;
			nd = &intersection[0].second->normalA;
			ud = &intersection[0].second->uvA;
			e = &std::get<0>(intersection[0].first);
			ne = &std::get<1>(intersection[0].first);
			ue = &std::get<2>(intersection[0].first);
		}
		else/* if (intersection[0].second->endB == intersection[1].second->endB)*/ {
			a = &intersection[0].second->endB;
			na = &intersection[0].second->normalB;
			ua = &intersection[0].second->uvB;
			b = &std::get<0>(intersection[0].first);
			nb = &std::get<1>(intersection[0].first);
			ub = &std::get<2>(intersection[0].first);
			c = &intersection[0].second->endA;
			nc = &intersection[0].second->normalA;
			uc = &intersection[0].second->uvA;
			d = &intersection[1].second->endA;
			nd = &intersection[1].second->normalA;
			ud = &intersection[1].second->uvA;
			e = &std::get<0>(intersection[1].first);
			ne = &std::get<1>(intersection[1].first);
			ue = &std::get<2>(intersection[1].first);
		}
		/*else {
			std::cout << "Fuck up " << std::endl;
			return out;
		}*/
		emplaceWithRotation(normal, *a, *b, *e, *na, *nb, *ne, *ua, *ub, *ue, out);
		emplaceWithRotation(normal, *b, *c, *d, *nb, *nc, *nd, *ua, *uc, *ud, out);
		emplaceWithRotation(normal, *e, *b, *d, *ne, *nb, *nd, *ue, *ub, *ud, out);
		break;
	case 3:
		if (intersection[0].first == intersection[1].first) {
			emplaceWithRotation(
				normal,
				intersection[0].second->endA,
				std::get<0>(intersection[2].first),
				intersection[0].second->endB,
				intersection[0].second->normalA,
				std::get<1>(intersection[2].first),
				intersection[0].second->normalB,
				intersection[0].second->uvA,
				std::get<2>(intersection[2].first),
				intersection[0].second->uvB,
				out
			);
			emplaceWithRotation(
				normal,
				intersection[1].second->endA,
				std::get<0>(intersection[2].first),
				intersection[1].second->endB,
				intersection[1].second->normalA,
				std::get<1>(intersection[2].first),
				intersection[1].second->normalB,
				intersection[1].second->uvA,
				std::get<2>(intersection[2].first),
				intersection[1].second->uvB,
				out
			);
		}
		else if (intersection[0].first == intersection[2].first) {
			emplaceWithRotation(
				normal,
				intersection[0].second->endA,
				std::get<0>(intersection[1].first),
				intersection[0].second->endB,
				intersection[0].second->normalA,
				std::get<1>(intersection[1].first),
				intersection[0].second->normalB,
				intersection[0].second->uvA,
				std::get<2>(intersection[1].first),
				intersection[0].second->uvB,
				out
			);
			emplaceWithRotation(
				normal,
				intersection[2].second->endA,
				std::get<0>(intersection[1].first),
				intersection[2].second->endB,
				intersection[2].second->normalA,
				std::get<1>(intersection[1].first),
				intersection[2].second->normalB,
				intersection[0].second->uvA,
				std::get<2>(intersection[1].first),
				intersection[0].second->uvB,
				out
			);
		}
		else {
			emplaceWithRotation(
				normal,
				intersection[1].second->endA,
				std::get<0>(intersection[0].first),
				intersection[1].second->endB,
				intersection[1].second->normalA,
				std::get<1>(intersection[0].first),
				intersection[1].second->normalB,
				intersection[1].second->uvA,
				std::get<2>(intersection[0].first),
				intersection[1].second->uvB,
				out
			);
			emplaceWithRotation(
				normal,
				intersection[2].second->endA,
				std::get<0>(intersection[0].first),
				intersection[2].second->endB,
				intersection[2].second->normalA,
				std::get<1>(intersection[0].first),
				intersection[2].second->normalB,
				intersection[2].second->uvA,
				std::get<2>(intersection[0].first),
				intersection[2].second->uvB,
				out
			);
		}
	}
	return out;
}