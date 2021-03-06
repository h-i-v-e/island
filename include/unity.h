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

	struct Vector2Export {
		float x, y;
	};

	struct MotuOptions {
		float maxZ;
		float waterRatio;
		float slopeMultiplier;
		float coastalSlopeMultiplier;
		float noiseMultiplier;
	};

	struct Vector3ExportArray {
		const Vector3Export *data;
		int length;
	};

	struct Vector2ExportArray {
		const Vector2Export *data;
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

	struct ExportMeshWithUV {
		void *handle;
		Vector3ExportArray vertices, normals;
		TriangleExportArray triangles;
		Vector2ExportArray uv;
	};

	struct ExportMeshArray {
		ExportMesh *data;
		int length;
	};

	struct ExportMeshWithUVArray {
		ExportMeshWithUV *data;
		int length;
	};

	struct ExportTextures {
		const uint32_t *texture;
		const uint32_t *albedo;
	};

	struct ExportHeightMap {
		int width, height;
		float* data;
	};

	struct ExportHeightMapWithSeaLevel {
		int width, height;
		float* data;
		float seaLevel;
	};

	struct ExportRiverNodes {
		TriangleExportArray *rivers;
		int length;
	};

	struct ExportDecoration {
		Vector3ExportArray trees, bushes, rocks;
	};

	struct TreeMeshPrototype {
		int offset;
		float scale;
	};

	struct TreeMeshPrototypes {
		TreeMeshPrototype *prototypes;
		int length;
	};

	struct ExportTreeBillboards {
		ExportMesh mesh;
		int *offsets;
	};

	struct ExportTreeBillboardsArray {
		ExportTreeBillboards octants[8];
		void *offsetsHandle;
	};

	EXPORT_TO_UNITY void *CreateMotu(int seed, const MotuOptions *);

	EXPORT_TO_UNITY void *LoadMotu(const char *filePath);

	EXPORT_TO_UNITY void SaveMotu(void *, const char *filePath);

	EXPORT_TO_UNITY void GetDecoration(void *, ExportDecoration *);

	EXPORT_TO_UNITY uint8_t *CreateNormalMap3DC(void *, int lod, int dimension);

	EXPORT_TO_UNITY void ReleaseNormalMap3DC(uint8_t *);

	EXPORT_TO_UNITY uint8_t *CreateNormalMap(void *, int lod, int dimension);

	EXPORT_TO_UNITY void ReleaseNormalMap(uint8_t *);

	EXPORT_TO_UNITY void ReleaseMotu(void *);

	EXPORT_TO_UNITY void CreateMesh(void *, const ExportArea *, int lod, uint8_t clampSides, ExportMesh *);

	EXPORT_TO_UNITY ExportHeightMapWithSeaLevel* CreateHeightMap(void *, int resolution);

	EXPORT_TO_UNITY void ReleaseHeightMap(ExportHeightMapWithSeaLevel *);

	EXPORT_TO_UNITY void CreateTreeBillboards(void *, TreeMeshPrototypes *input, ExportTreeBillboardsArray *);

	EXPORT_TO_UNITY void ReleaseTreeBillboards(ExportTreeBillboardsArray *);

	EXPORT_TO_UNITY void ReleaseMeshes(ExportMeshArray *output);

	EXPORT_TO_UNITY void ReleaseMesh(ExportMesh *);

	EXPORT_TO_UNITY void CreateRiverMesh(void *, const ExportArea *, ExportMeshWithUV *);

	EXPORT_TO_UNITY void ReleaseMeshWithUV(ExportMeshWithUV *);

	EXPORT_TO_UNITY uint32_t *ExportFoliageData(void *, int dimension);

	EXPORT_TO_UNITY void ReleaseFoliageData(uint32_t *);

	EXPORT_TO_UNITY float* CreateSeaDepthMap(void *ptr, int dimension);

	EXPORT_TO_UNITY void ReleaseSeaDepthMap(float *);

	EXPORT_TO_UNITY void SetLogFile(const char *path);
}

#endif