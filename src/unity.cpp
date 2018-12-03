#include <random>
#include <fstream>

#include "unity.h"
#include "island.h"
#include "matrix4.h"
#include "bounding_box.h"
#include "height_map.h"
#include "mesh_triangle_map.h"
#include "mesh_edge_map.h"
#include "river_quantizer.h"
#include "pipe_erosian.h"
#include "normal_map_compressor.h"
#include "logger.h"

using namespace motu;

namespace {
	struct DecorationHandle {
		std::vector<Vector3> trees, bushes, rocks;
	};

	void ExportVector2Vector(const std::vector<Vector2> &vec, Vector2ExportArray &exp) {
		exp.data = reinterpret_cast<const Vector2Export*>(vec.data());
		exp.length = static_cast<int>(vec.size());
	}

	void ExportVector3Vector(const std::vector<Vector3> &vec, Vector3ExportArray &exp) {
		exp.data = reinterpret_cast<const Vector3Export*>(vec.data());
		exp.length = static_cast<int>(vec.size());
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

	inline uint32_t clamp(float in) {
		float out = in * 255.0f;
		if (out > 255.0f) {
			return 255;
		}
		if (out < 0.0f) {
			return 0.0f;
		}
		return static_cast<uint32_t>(out);
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

ExportHeightMapWithSeaLevel *CreateHeightMap(void *motu, int resolution) {
	Island *island = reinterpret_cast<Island*>(motu);
	Mesh mesh;
	island->lod(0).slice(BoundingBox(Vector3(0.0f, 0.0f, -0.02f), Vector3(1.0f, 1.0f, 1.0f)), mesh);
	HeightMap *hm = new HeightMap(resolution, resolution);
	hm->load(mesh);
	return reinterpret_cast<ExportHeightMapWithSeaLevel*>(hm);
}

void ReleaseHeightMap(ExportHeightMapWithSeaLevel *map) {
	delete reinterpret_cast<HeightMap*>(map);
}

void CreateRiverMesh(void *motu, const ExportArea *ea, ExportMesh *em) {
	Island *island = reinterpret_cast<Island*>(motu);
	const BoundingBox &bb = *reinterpret_cast<const BoundingBox*>(ea);
	Mesh *out = new Mesh;
	island->riverMesh().slice(bb, *out);
	ExportAMesh(*em, out);
}

void ReleaseRiverMesh(ExportMesh *ema) {
	delete reinterpret_cast<Mesh*>(ema->handle);
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
	DecorationHandle *handle = new DecorationHandle();
	const Mesh &lod0 = island->lod(0);
	MeshEdgeMap mem(lod0);
	ExportVector3Vector(decoration.getTrees(lod0, mem, handle->trees), exp->trees);
	ExportVector3Vector(decoration.getBushes(lod0, handle->bushes), exp->bushes);
	ExportVector3Vector(decoration.getRocks(lod0, mem, handle->rocks), exp->rocks);
	exp->data = handle;
}

void ReleaseDecoration(ExportDecoration *ptr) {
	delete reinterpret_cast<DecorationHandle*>(ptr->data);
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

uint32_t *ExportFoliageData(void *motu, int dimension) {
	uint32_t *output = new uint32_t[dimension * dimension];
	const Island *island = reinterpret_cast<Island*>(motu);
	Mesh buffer;
	Grid<float> grid(dimension, dimension);
	buffer = island->decoration().mesh;
	for (size_t i = 0; i != buffer.vertices.size(); ++i) {
		buffer.vertices[i].z = island->decoration().soilRichness[i];
	}
	buffer.rasterize(grid);
	for (int i = 0, j = grid.length(); i != j; ++i) {
		output[i] = clamp(grid.data()[i]) << 24;
	}
	std::fill(grid.data(), grid.data() + grid.length(), 0.0f);
	for (size_t i = 0; i != buffer.vertices.size(); ++i) {
		buffer.vertices[i].z = island->decoration().forest[i];
	}
	buffer.rasterize(grid);
	for (int i = 0, j = grid.length(); i != j; ++i) {
		output[i] |= clamp(grid.data()[i]) << 16;
	}
	return output;
}

void ReleaseFoliageData(uint32_t *data) {
	delete[] data;
}

uint8_t *CreateNormalMap3DC(void *motu, int lod, int dimension) {
	/*Island::Image nao(dimension, dimension);
	const Island *island = reinterpret_cast<Island*>(motu);
	island->generateNormalMap(lod, nao);
	const uint8_t *in = reinterpret_cast<uint8_t *>(nao.data());
	uint8_t *output = new uint8_t[dimension * dimension];
	int length;
	CompressNormalMap3Dc(in, output, dimension, dimension, length);
	return output;*/
	return nullptr;
}

void ReleaseNormalMap3DC(uint8_t *data) {
	//delete[] data;
}

uint8_t *CreateNormalMap(void *motu, int lod, int dimension) {
	Island::Image24 nao(dimension, dimension);
	const Island *island = reinterpret_cast<Island*>(motu);
	island->generateNormalMap(lod, nao);
	return reinterpret_cast<uint8_t*>(nao.detach());
}

void ReleaseNormalMap(uint8_t *data) {
	delete[] reinterpret_cast<Colour24*>(data);
}

float* CreateSeaDepthMap(void *ptr, int dimension) {
	size_t len = dimension * dimension;
	Island *island = reinterpret_cast<Island*>(ptr);
	HeightMap hm(dimension, dimension);
	Mesh buffer;
	int lod;
	if (dimension <= 1024) {
		lod = dimension <= 256 ? 2 : 1;
	}
	else {
		lod = 0;
	}
	island->lod(lod).slice(BoundingBox(0.0f, 0.0f, -25.5f / Island::size(), 1.0f, 1.0f, 1.0f), buffer);
	for (Vector3 &vec : buffer.vertices) {
		if (vec.z > 0.0f) {
			vec.z = 0.0f;
		}
	}
	hm.load(buffer);
	hm.normalise();
	return hm.detach();
}

void CreateTreeBillboards(void *ptr, TreeMeshPrototypes *input, ExportTreeBillboardsArray *output) {
	Island *island = reinterpret_cast<Island*>(ptr);
	int len = input->length;
	TreeBillboards::Positions positions(len);
	//TreeBillboards::Offsets offsets(len);
	const Decoration &decoration = island->decoration();
	for (int i = 0; i != len; ++i) {
		auto &proto = input->prototypes[i];
		auto &pos = positions[i];
		pos.position = island->lod(0).vertices[decoration.trees[proto.offset]];
		pos.scale = proto.scale;
	}
	std::vector<TreeBillboards::UPtr> octants;
	TreeBillboards::OctantOffsets *octantOffsets = new TreeBillboards::OctantOffsets;
	TreeBillboards::createOctants(positions, octants, *octantOffsets);
	for (size_t i = 0; i != 8; ++i) {
		ExportAMesh(output->octants[i].mesh, octants[i].release());
		output->octants[i].offsets = (*octantOffsets)[i]->data();
	}
	output->offsetsHandle = octantOffsets;
}

void ReleaseTreeBillboards(ExportTreeBillboardsArray *billboards) {
	for (size_t i = 0; i != 8; ++i) {
		ReleaseMesh(&billboards->octants[i].mesh);
	}
	delete reinterpret_cast<TreeBillboards::OctantOffsets*>(billboards->offsetsHandle);
}

void ReleaseMeshes(ExportMeshArray *output) {
	for (auto i = output->data, j = output->data + output->length; i != j; ++i) {
		ReleaseMesh(i);
	}
	delete[] output->data;
}

void ReleaseSeaDepthMap(float *ptr) {
	delete[] ptr;
}

void *LoadMotu(const char *filePath) {
	std::ifstream in(filePath, std::ios::binary | std::ios::in);
	Island *out = new Island;
	in >> *out;
	return out;
}

void SaveMotu(void *island, const char *filePath) {
	std::ofstream out(filePath, std::ios::binary | std::ios::out);
	out << *reinterpret_cast<const Island*>(island);
}

/*uint32_t *CreateNormalAndOcclusianMap(void *motu, int lod, int dimension) {
	Island::Image nao(dimension, dimension);
	const Island *island = reinterpret_cast<Island*>(motu);
	island->generateNormalAndOcclusianMap(nao);
	return nao.detach();
}

void ReleaseNormalAndOcclusianMap(uint32_t *ptr) {
	delete[] ptr;
}*/

void SetLogFile(const char *path) {
	motu::setLogFile(path);
}