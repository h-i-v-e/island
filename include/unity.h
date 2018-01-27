#ifndef UNITY_H
#define UINTY_H

#include <cstdint>

extern "C" {
	struct Motu {
		void *data;
		uint32_t *normalMap;
		uint8_t *occlusianMap;
	};

	struct MotuMesh {
		void *data;
		float *vertices;
		uint32_t numVertices;
		uint32_t *triangles;
		uint32_t numTriangles;
	};

	MotuMesh *MotuGetMesh(Motu *, int lod, float ux, float uy, float uz, float lx, float ly, float lz);

	void MotuFreeMesh(MotuMesh *mesh);

	Motu *MotuAllocateIsland(int seed);

	void MotuReleaseIsland(Motu *);
}

#endif