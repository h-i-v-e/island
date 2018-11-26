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

	struct ExportQuantizedRiverNode {
		int x, y, flow;
	};

	struct ExportQuantisedRiver {
		ExportMesh mesh;
		Vector2ExportArray uv;
		ExportQuantizedRiverNode *quantisedRiverNodes;
		int numNodes;
	};

	struct ExportQuantisedRiverArray {
		ExportQuantisedRiver *data;
		int length;
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
		void *data;
		Vector3ExportArray trees, bushes, rocks;// bigRocks, mediumRocks, smallRocks, forestScatter;
	};

	EXPORT_TO_UNITY void *CreateMotu(int seed, const MotuOptions *);

	EXPORT_TO_UNITY void *LoadMotu(const char *filePath);

	EXPORT_TO_UNITY void SaveMotu(void *, const char *filePath);

	EXPORT_TO_UNITY void GetDecoration(void *, ExportDecoration *);

	EXPORT_TO_UNITY void ReleaseDecoration(ExportDecoration *);

	EXPORT_TO_UNITY uint8_t *CreateNormalMap3DC(void *, int lod, int dimension);

	EXPORT_TO_UNITY void ReleaseNormalMap3DC(uint8_t *);

	EXPORT_TO_UNITY uint8_t *CreateNormalMap(void *, int lod, int dimension);

	EXPORT_TO_UNITY void ReleaseNormalMap(uint8_t *);

	//EXPORT_TO_UNITY void FetchTextures(void *, ExportTextures *);

	EXPORT_TO_UNITY void ReleaseMotu(void *);

	EXPORT_TO_UNITY void CreateMesh(void *, const ExportArea *, int lod, uint8_t clampSides, ExportMesh *);

	EXPORT_TO_UNITY ExportHeightMapWithSeaLevel* CreateHeightMap(void *, int resolution);

	EXPORT_TO_UNITY void ReleaseHeightMap(ExportHeightMapWithSeaLevel *);

	EXPORT_TO_UNITY void ReleaseMesh(ExportMesh *);

	EXPORT_TO_UNITY void CreateRiverMeshes(void *, ExportMeshArray *);

	EXPORT_TO_UNITY void ReleaseRiverMeshes(ExportMeshArray *);

	EXPORT_TO_UNITY void CreateForestMeshesLod1(void *, int tilesPerAxis, float treeHeight, ExportMeshWithUVArray *);

	EXPORT_TO_UNITY void ReleaseForestMeshesLod1(ExportMeshWithUVArray *out);

	/*EXPORT_TO_UNITY void CreateRiverNodes(void *, ExportRiverNodes *, const ExportHeightMap *);

	EXPORT_TO_UNITY void ReleaseRiverNodes(ExportRiverNodes *);*/

	EXPORT_TO_UNITY void CreateQuantisedRiverMeshes(void *, ExportHeightMapWithSeaLevel *, ExportQuantisedRiverArray *);

	EXPORT_TO_UNITY void ReleaseQuantisedRiverMeshes(ExportQuantisedRiverArray *);

	EXPORT_TO_UNITY ExportHeightMap* CreateSoilRichnessMap(void *, int resolution);

	EXPORT_TO_UNITY ExportHeightMap* CreateForestMap(void *, int resolution);

	EXPORT_TO_UNITY void ReleaseFoliageMap(ExportHeightMap *);

	EXPORT_TO_UNITY uint32_t *ExportFoliageData(void *, int dimension);

	EXPORT_TO_UNITY void ReleaseFoliageData(uint32_t *);

	EXPORT_TO_UNITY float* CreateSeaDepthMap(void *ptr, int dimension);

	EXPORT_TO_UNITY void ReleaseSeaDepthMap(float *);
}

#endif