#include <random>

#include "unity.h"
#include "island.h"

using namespace motu;

void *CreateMotu(int seed, const MotuOptions *options) {
	return new Island(std::default_random_engine(seed), *reinterpret_cast<const Island::Options*>(options));
}

void ReleaseMotu(void *ptr) {
	delete reinterpret_cast<Island*>(ptr);
}

void CreateMesh(void *ptr, const ExportArea *area, int lod, ExportMesh *exp) {
	Island *island = reinterpret_cast<Island*>(ptr);
	Mesh *mesh = new Mesh;
	island->lod(lod).slice(*reinterpret_cast<const BoundingBox*>(area), *mesh);
	exp->handle = mesh;
	exp->normals.data = reinterpret_cast<Vector3Export*>(mesh->normals.data());
	exp->normals.length = static_cast<int>(mesh->normals.size());
	exp->vertices.data = reinterpret_cast<Vector3Export*>(mesh->vertices.data());
	exp->vertices.length = static_cast<int>(mesh->vertices.size());
	exp->triangles.data = reinterpret_cast<int*>(mesh->triangles.data());
	exp->triangles.length = static_cast<int>(mesh->triangles.size());
}

void ReleaseMesh(ExportMesh *exp) {
	delete reinterpret_cast<Mesh*>(exp->handle);
}

void FetchTextures(void *motu, ExportTextures *et) {
	Island *island = reinterpret_cast<Island*>(motu);
	et->texture = island->normalAndOcclusianMap().data();
	et->albedo = island->albedoMap().data();
}