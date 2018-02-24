#include <random>

#include "unity.h"
#include "island.h"

using namespace motu;

void *CreateMotu(int seed) {
	Island::Options options;
	options.pallete.cliff = Vector3(0.9f, 0.9f, 0.9f);
	options.pallete.sand = Vector3(1.0f, 0.95f, 0.9f);
	options.pallete.grass = Vector3(0.5f, 1.0f, 0.35f);
	options.pallete.mountain = Vector3(1.0f, 0.9f, 0.25f);
	return new Island(std::default_random_engine(seed), options);
}

void ReleaseMotu(void *ptr) {
	delete reinterpret_cast<Island*>(ptr);
}

void CreateMesh(void *ptr, const ExportArea *area, int lod, ExportMesh *exp) {
	Island *island = reinterpret_cast<Island*>(ptr);
	Mesh *mesh = new Mesh;
	island->lod(lod).slice(*reinterpret_cast<const BoundingBox*>(area), *mesh);
	mesh->calculateNormals();
	exp->handle = mesh;
	exp->normals.data = reinterpret_cast<Vector3Export*>(mesh->normals.data());
	exp->normals.length = mesh->normals.size();
	exp->vertices.data = reinterpret_cast<Vector3Export*>(mesh->vertices.data());
	exp->vertices.length = mesh->vertices.size();
	exp->triangles.data = reinterpret_cast<int*>(mesh->triangles.data());
	exp->triangles.length = mesh->triangles.size();
}

void ReleaseMesh(ExportMesh *exp) {
	delete reinterpret_cast<Mesh*>(exp->handle);
}

void FetchTextures(void *motu, ExportTextures *et) {
	Island *island = reinterpret_cast<Island*>(motu);
	et->texture = island->normalAndOcclusianMap().data();
	et->albedo = island->albedoMap().data();
}