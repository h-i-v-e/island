#include "unity.h"
#include "island.h"
#include "bounding_box.h"

using namespace motu;

MotuMesh *MotuGetMesh(Motu *motu, int lod, float ux, float uy, float uz, float lx, float ly, float lz) {
	Mesh *mesh = new Mesh();
	reinterpret_cast<Island*>(motu->data)->lod(lod).slice(BoundingBox(ux, uy, uz, lx, ly, lz), *mesh);
	MotuMesh *out = new MotuMesh;
	out->data = mesh;
	out->numTriangles = mesh->triangles.size();
	out->numVertices = mesh->vertices.size();
	out->triangles = mesh->triangles.data();
	out->vertices = reinterpret_cast<float*>(mesh->vertices.data());
	return out;
}

void MotuFreeMesh(MotuMesh *mesh) {
	delete reinterpret_cast<Mesh*>(mesh->data);
	delete mesh;
}

Motu *MotuAllocateIsland(int seed) {
	Motu *motu = new Motu;
	std::default_random_engine rand(seed);
	Island *island = new Island(rand, 4096, 2, 0.05f, 0.5f);
	motu->data = island;
	//motu->normalMap = const_cast<uint32_t*>(island->normalMap().data());
	//motu->occlusianMap = const_cast<uint8_t*>(island->occlusianMap().data());
	return motu;
}

void MotuReleaseIsland(Motu *motu) {
	delete reinterpret_cast<Island*>(motu->data);
	delete motu;
}