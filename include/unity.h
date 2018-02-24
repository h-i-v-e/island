#ifndef UNITY_H
#define UINTY_H

#include <cstdint>
#ifdef _WIN32
#define EXPORT_TO_UNITY __declspec(dllexport)
#else
#define EXPORT_TO_UNITY
#endif

extern "C" {

	struct Vector3Export {
		float x, y, z;
	};

	struct Vector3ExportArray {
		const Vector3Export *data;
		int length;
	};

	struct TriangleExportArray {
		const int *data;
		int length;
	};

	struct ExportArea {
		Vector3Export min, max;
	};

	struct ExportMesh {
		void *handle;
		Vector3ExportArray vertices, normals;
		TriangleExportArray triangles;
	};

	struct ExportTextures {
		const uint32_t *texture;
		const uint16_t *albedo;
	};

	EXPORT_TO_UNITY void *CreateMotu(int seed);

	EXPORT_TO_UNITY void FetchTextures(void *, ExportTextures *);

	EXPORT_TO_UNITY void ReleaseMotu(void *);

	EXPORT_TO_UNITY void CreateMesh(void *, const ExportArea *, int lod, ExportMesh *);

	EXPORT_TO_UNITY void ReleaseMesh(ExportMesh *);
}

#endif