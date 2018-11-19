#ifndef MOTU_NORMAL_MAP_COMPRESSOR
#define MOTU_NORMAL_MAP_COMPRESSOR

#include <cstdint>

void CompressNormalMap3Dc(const uint8_t *inBuf, uint8_t *outBuf, int width, int height, int &outputBytes);
#endif