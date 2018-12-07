#include <random>
#include <fstream>

#include "unity.h"
#include "island.h"
#include "matrix4.h"
#include "bounding_box.h"
#include "height_map.h"
#include "mesh_triangle_map.h"
#include "mesh_edge_map.h"
#include "pipe_erosian.h"
#include "normal_map_compressor.h"
#include "logger.h"

using namespace motu;

namespace {

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

	void ExportAMeshWithUV(ExportMeshWithUV &exp, MeshWithUV *mesh) {
		ExportAMesh(*reinterpret_cast<ExportMesh*>(&exp), reinterpret_cast<Mesh*>(mesh));
		exp.uv.length = static_cast<int>(mesh->uv.size());
		exp.uv.data = reinterpret_cast<Vector2Export*>(mesh->uv.data());
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

void CreateRiverMesh(void *motu, const ExportArea *ea, ExportMeshWithUV *em) {
	Island *island = reinterpret_cast<Island*>(motu);
	const BoundingBox &bb = *reinterpret_cast<const BoundingBox*>(ea);
	MeshWithUV *out = new MeshWithUV;
	island->riverMesh().slice(bb, *out);
	ExportAMeshWithUV(*em, out);
}

void ReleaseMeshWithUV(ExportMeshWithUV *ema){
	delete reinterpret_cast<MeshWithUV*>(ema->handle);
}

void GetDecoration(void *motu, ExportDecoration *exp) {
	const Island *island = reinterpret_cast<Island*>(motu);
	const Decoration &decoration = island->decoration();
	ExportVector3Vector(decoration.trees(), exp->trees);
	ExportVector3Vector(decoration.bushes(), exp->bushes);
	ExportVector3Vector(decoration.rocks() , exp->rocks);
}

uint32_t *ExportFoliageData(void *motu, int dimension) {
	uint32_t *output = new uint32_t[dimension * dimension];
	const Island *island = reinterpret_cast<Island*>(motu);
	Grid<float> grid(dimension, dimension);
	const Decoration &decoration = island->decoration();
	decoration.fillForestMap(island->lod(0), grid);
	for (int i = 0, j = grid.length(); i != j; ++i) {
		output[i] = clamp(grid.data()[i]) << 24;
	}
	std::fill(grid.data(), grid.data() + grid.length(), 0.0f);
	decoration.fillRiverMap(island->lod(0), grid);
	for (int i = 0, j = grid.length(); i != j; ++i) {
		output[i] |= clamp(grid.data()[i]) << 16;
	}
	std::fill(grid.data(), grid.data() + grid.length(), 0.0f);
	decoration.fillRockMap(island->lod(0), grid);
	for (int i = 0, j = grid.length(); i != j; ++i) {
		output[i] |= clamp(grid.data()[i]) << 8;
	}
	std::fill(grid.data(), grid.data() + grid.length(), 0.0f);
	decoration.fillSoilRichnessMap(island->lod(0), grid);
	for (int i = 0, j = grid.length(); i != j; ++i) {
		output[i] |= clamp(grid.data()[i]);
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
	const Decoration &decoration = island->decoration();
	for (int i = 0; i != len; ++i) {
		auto &proto = input->prototypes[i];
		auto &pos = positions[i];
		pos.position = decoration.trees()[proto.offset];
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

void SetLogFile(const char *path) {
	motu::setLogFile(path);
}