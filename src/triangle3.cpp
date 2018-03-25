#include "triangle3.h"

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
		const Vector3 &na, const Vector3 &nb, const Vector3 &nc, std::vector<Triangle3WithNormals> &out) {
		if ((b - a).cross(c - b).dot(normal) < 0.0f) {
			out.emplace_back(c, b, a, nc, nb, na);
		}
		else {
			out.emplace_back(a, b, c, na, nb, nc);
		}
	}
}

bool Triangle3::intersection(const Spline &spline, Vector3 &intersection) const {
	Plane plane(vertices[0], normal());
	if (plane.intersection(spline.endA, spline.direction(), intersection)) {
		float a = (vertices[1] - vertices[0]).cross(intersection - vertices[0]).dot(plane.normal),
			b = (vertices[2] - vertices[1]).cross(intersection - vertices[1]).dot(plane.normal);
		if ((a < 0.0f && b > 0.0f) || (a > 0.0f && b < 0.0f)) {
			return false;
		}
		b = (vertices[0] - vertices[2]).cross(intersection - vertices[2]).dot(plane.normal);
		return (a > 0.0f && b > 0.0f) || (a < 0.0f && b < 0.0f);
	}
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
		else if (intersection[0].second->endB == intersection[1].second->endB) {
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
		else if (intersection[0].second->endB == intersection[1].second->endB) {
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
		emplaceWithRotation(normal, *a, *b, *e, *na, *nb, *nc, out);
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