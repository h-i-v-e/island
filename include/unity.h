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
		float maxRiverGradient;
		float riverDepth;
		float riverSourceSDThreshold;
		float maxZ;
		float waterRatio;
		float slopeMultiplier;
		float coastalSlopeMultiplier;
		float noiseMultiplier;
		int erosianPasses;
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

	struct ExportMeshArray{
		ExportMesh *data;
		int length;
	};

	struct ExportTextures {
		const uint32_t *texture;
		const uint32_t *albedo;
	};

	struct ExportHeightMap {
		int width, height;
		float* data;
		float seaLevel;
	};

	struct ExportRiverNodes {
		TriangleExportArray *rivers;
		int length;
	};

	EXPORT_TO_UNITY void *CreateMotu(int seed, const MotuOptions *);

	//EXPORT_TO_UNITY void FetchTextures(void *, ExportTextures *);

	EXPORT_TO_UNITY void ReleaseMotu(void *);

	EXPORT_TO_UNITY void CreateMesh(void *, const ExportArea *, int lod, uint8_t clampSides, ExportMesh *);

	EXPORT_TO_UNITY ExportHeightMap* CreateHeightMap(void *, int resolution);

	EXPORT_TO_UNITY void ReleaseHeightMap(ExportHeightMap *);

	EXPORT_TO_UNITY void ReleaseMesh(ExportMesh *);

	EXPORT_TO_UNITY void CreateRiverMeshes(void *, ExportMeshArray *);

	EXPORT_TO_UNITY void ReleaseRiverMeshes(ExportMeshArray *);

	/*EXPORT_TO_UNITY void CreateRiverNodes(void *, ExportRiverNodes *, const ExportHeightMap *);

	EXPORT_TO_UNITY void ReleaseRiverNodes(ExportRiverNodes *);*/

	EXPORT_TO_UNITY void CreateQuantisedRiverMeshes(void *, ExportHeightMap *, ExportQuantisedRiverArray *);

	EXPORT_TO_UNITY void ReleaseQuantisedRiverMeshes(ExportQuantisedRiverArray *);
}

#endif