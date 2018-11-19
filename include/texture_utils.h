#ifndef MOTU_TEXTURE_DETAILS
#define MOTU_TEXTURE_DETAILS

#include <cstdint>

#include "grid.h"
#include "colour.h"

namespace motu {
	struct Mesh;

	typedef Grid<Colour24> Image;

	//void GenerateNormalAndOcclusianMap(const Mesh &highDetail, const Mesh &lowDetail, Image &output);

	void GenerateNormalMap(const Mesh &highDetail, const Mesh &lowDetail, Image &output);
}

#endif