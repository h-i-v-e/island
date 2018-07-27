#include <random>

#include "unity.h"
#include "island.h"
#include "matrix4.h"
#include "bounding_box.h"
#include "height_map.h"
#include "mesh_triangle_map.h"
#include "mesh_edge_map.h"
#include "river_quantizer.h"


using namespace motu;

namespace {
	void ExportVector2Vector(const std::vector<Vector2> &vec, Vector2ExportArray &exp) {
		exp.data = reinterpret_cast<const Vector2Export*>(vec.data());
		exp.length = vec.size();
	}

	void ExportAMesh(ExportMesh &exp, Mesh *mesh) {
		exp.handle = mesh;
		if (!mesh) {
			exp.normals.data = nullptr;
			exp.normals.length = 0;
			exp.vertices.data = nullptr;
			exp.vertices.length = 0;
			exp.triangles.data = nullptr;
			exp.triangles.length = 0;
			return;
		}
		exp.normals.data = reinterpret_cast<Vector3Export*>(mesh->normals.data());
		exp.normals.length = static_cast<int>(mesh->normals.size());
		exp.vertices.data = reinterpret_cast<Vector3Export*>(mesh->vertices.data());
		exp.vertices.length = static_cast<int>(mesh->vertices.size());
		exp.triangles.data = reinterpret_cast<int*>(mesh->triangles.data());
		exp.triangles.length = static_cast<int>(mesh->triangles.size());
	}

	Grid<float> &createHeightMap(const Mesh &mesh, const std::vector<float> &z, Grid<float> &out) {
		Mesh buf = mesh;
		for (size_t i = 0; i != z.size(); ++i) {
			buf.vertices[i].z = z[i];
		}
		buf.rasterize(out);
		return out;
	}
}

void *CreateMotu(int seed, const MotuOptions *options) {
	return new Island(std::default_random_engine(seed), *reinterpret_cast<const Island::Options*>(options));
}

void ReleaseMotu(void *ptr) {
	delete reinterpret_cast<Island*>(ptr);
}

void CreateMesh(void *ptr, const ExportArea *area, int lod, uint8_t clampSides, ExportMesh *exp) {
	Island *island = reinterpret_cast<Island*>(ptr);
	Mesh *mesh = new Mesh;
	if (lod == 2) {
		island->lod(lod).slice(*reinterpret_cast<const BoundingBox*>(area), *mesh);
	}
	else {
		island->lod(lod).slice(*reinterpret_cast<const BoundingBox*>(area), clampSides, island->lod(lod + 1), *mesh);
	}
	ExportAMesh(*exp, mesh);
}

void ReleaseMesh(ExportMesh *exp) {
	delete reinterpret_cast<Mesh*>(exp->handle);
}

/*void FetchTextures(void *motu, ExportTextures *et) {
	Island *island = reinterpret_cast<Island*>(motu);
	et->texture = island->normalAndOcclusianMap().data();
	et->albedo = island->albedoMap().data();
}*/

ExportHeightMapWithSeaLevel *CreateHeightMap(void *motu, int resolution) {
	Island *island = reinterpret_cast<Island*>(motu);
	Mesh mesh;
	island->lod(0).slice(BoundingBox(Vector3(0.0f, 0.0f, -0.02f), Vector3(1.0f, 1.0f, 1.0f)), mesh);
	HeightMap *hm = new HeightMap(resolution, resolution);
	hm->load(mesh);
	for (int i = 0; i != 8; ++i) {
		hm->smooth();
	}
	return reinterpret_cast<ExportHeightMapWithSeaLevel*>(hm);
}

void ReleaseHeightMap(ExportHeightMapWithSeaLevel *map) {
	delete reinterpret_cast<HeightMap*>(map);
}

void CreateRiverMeshes(void *motu, ExportMeshArray *ema) {
	Island *island = reinterpret_cast<Island*>(motu);
	int num = static_cast<int>(island->riverMeshes().size());
	ema->data = new ExportMesh[num];
	ema->length = num;
	for (int i = 0; i != num; ++i) {
		ExportAMesh(ema->data[i], &*island->riverMeshes()[i]);
	}
}

void ReleaseRiverMeshes(ExportMeshArray *ema) {
	delete[] ema->data;
}

/*void CreateRiverNodes(void *handle, ExportRiverNodes *nodes, const ExportHeightMap *ehm) {
	Island *island = reinterpret_cast<Island*>(handle);
	std::vector<std::pair<int, int>> output;
	int len = island->riverVertexLists().size();
	nodes->length = len;
	nodes->rivers = new TriangleExportArray[len];
	for (int i = 0; i != len; ++i) {
		const auto &list = *island->riverVertexLists()[i];
		output.clear();
		quantizeRiver(*reinterpret_cast<const HeightMap*>(ehm), list, output);
		int l = output.size();
		nodes->rivers[i].length = l << 1;
		int *data = new int[l << 1];
		for (int j = 0, k = 0; j != l; ++j) {
			std::pair<int, int> node = output[j];
			data[k++] = static_cast<int>(node.first);
			data[k++] = static_cast<int>(node.second);
		}
		nodes->rivers[i].data = data;
	}
}*/

/*void ReleaseRiverNodes(ExportRiverNodes *nodes) {
	int len = nodes->length;
	for (int i = 0; i != len; ++i) {
		delete[] nodes->rivers[i].data;
	}
	delete[] nodes->rivers;
}*/

void CreateQuantisedRiverMeshes(void *motu, ExportHeightMapWithSeaLevel *ehm, ExportQuantisedRiverArray *exp) {
	const Island *island = reinterpret_cast<Island*>(motu);
	HeightMap *hm = reinterpret_cast<HeightMap*>(ehm);
	size_t len = island->riverVertexLists().size();
	exp->length = static_cast<int>(len);
	exp->data = new ExportQuantisedRiver[len];
	std::vector<QuantisedRiver> outputs(len);
	RiverQuantiser rq(*hm);
	for (size_t i = 0; i != len; ++i) {
		outputs[i].reserve(island->riverVertexLists()[i]->size());
		rq.quantiseRiver(*island->riverVertexLists()[i], outputs[i]);
	}
	std::vector<QuantisedRiver*> buffer(len);
	std::vector<int> joinTos;
	for (size_t i = 0; i != len; ++i) {
		buffer[i] = outputs.data() + i;
	}
	dedupeQuantisedRivers(*hm, buffer, joinTos);
	int maxFlow = 0;
	for (const auto &i : outputs) {
		if (!i.empty() && i.back().flow > maxFlow) {
			maxFlow = i.back().flow;
		}
	}
	float mf = sqrtf(static_cast<float>(maxFlow));
	std::vector<MeshWithUV*> meshes;
	meshes.reserve(buffer.size());
	for (size_t i = 0; i != len; ++i) {
		QuantisedRiver *river = buffer[i];
		if (river->size() > 2) {
			MeshWithUV *mesh = new MeshWithUV;
			createQuantisedRiverMesh(*hm, *river, *mesh, mf);
			meshes.push_back(mesh);
		}
		else {
			meshes.push_back(nullptr);
		}
	}
	for (size_t i = 0; i != len; ++i) {
		if (joinTos[i] == -1) {
			continue;
		}
		MeshWithUV *from = meshes[i], *to = meshes[joinTos[i]];
		if (from && to && from->triangles.size() && to->triangles.size()) {
			joinQuantisedRiverMeshes(*hm, *from, *to);
		}
	}
	for (size_t i = 0; i != len; ++i) {
		MeshWithUV *mesh = meshes[i];
		ExportAMesh(exp->data[i].mesh, mesh);
		QuantisedRiver *river = buffer[i];
		if (mesh) {
			exp->data[i].uv.length = static_cast<int>(mesh->uv.size());
			exp->data[i].uv.data = reinterpret_cast<Vector2Export*>(mesh->uv.data());
		}
		else {
			exp->data[i].uv.length = 0;
			exp->data[i].uv.data = nullptr;
		}
		exp->data[i].quantisedRiverNodes = new ExportQuantizedRiverNode[river->size()];
		exp->data[i].numNodes = static_cast<int>(river->size());
		for (size_t j = 0; j != river->size(); ++j) {
			exp->data[i].quantisedRiverNodes[j] = *reinterpret_cast<ExportQuantizedRiverNode*>(river->data() + j);
		}
	}
}

void ReleaseQuantisedRiverMeshes(ExportQuantisedRiverArray *exp) {
	for (auto i = exp->data, j = exp->data + exp->length; i != j; ++i) {
		delete reinterpret_cast<MeshWithUV *>(i->mesh.handle);
		delete[] i->quantisedRiverNodes;
	}
	delete[] exp->data;
}

void GetDecoration(void *motu, ExportDecoration *exp) {
	const Island *island = reinterpret_cast<Island*>(motu);
	const Decoration &decoration = island->decoration();
	ExportVector2Vector(decoration.trees, exp->trees);
	ExportVector2Vector(decoration.bushes, exp->bushes);
	ExportVector2Vector(decoration.bigRocks, exp->bigRocks);
	ExportVector2Vector(decoration.mediumRocks, exp->mediumRocks);
	ExportVector2Vector(decoration.smallRocks, exp->smallRocks);
}

ExportHeightMap* CreateSoilRichnessMap(void *motu, int resolution) {
	const Island *island = reinterpret_cast<Island*>(motu);
	Grid<float> *grid = new Grid<float>(resolution, resolution);
	createHeightMap(island->decoration().mesh, island->decoration().soilRichness, *grid);
	return reinterpret_cast<ExportHeightMap*>(grid);
}

ExportHeightMap* CreateForestMap(void *motu, int resolution) {
	const Island *island = reinterpret_cast<Island*>(motu);
	Grid<float> *grid = new Grid<float>(resolution, resolution);
	createHeightMap(island->decoration().mesh, island->decoration().forest, *grid);
	return reinterpret_cast<ExportHeightMap*>(grid);
}

void ReleaseFoliageMap(ExportHeightMap *ehm) {
	delete reinterpret_cast<Grid<float>*>(ehm);
}