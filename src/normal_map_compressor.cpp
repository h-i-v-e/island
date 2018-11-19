/*
	Real-Time Normal Map Compression (C++)
	Copyright (C) 2008 Id Software, Inc.
	Written by J.M.P. van Waveren

	This code is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This code is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.
*/
#include "normal_map_compressor.h"

#include <cstring>

#define INSET_COLOR_SHIFT       4       // inset color channel
#define INSET_ALPHA_SHIFT       5       // inset alpha channel

#define C565_5_MASK             0xF8    // 0xFF minus last three bits
#define C565_6_MASK             0xFC    // 0xFF minus last two bits

namespace {
	void ExtractBlock(const uint8_t *inPtr, const int width, uint8_t *block) {
		for (int j = 0; j < 4; j++) {
			std::memcpy(&block[j * 4 * 4], inPtr, 4 * 4);
			inPtr += width * 4;
		}
	}

	void GetMinMaxNormalsBBox(const uint8_t *block, uint8_t *minNormal, uint8_t *maxNormal) {

		minNormal[0] = minNormal[1] = 255;
		maxNormal[0] = maxNormal[1] = 0;

		for (int i = 0; i < 16; i++) {
			if (block[i * 4 + 0] < minNormal[0]) {
				minNormal[0] = block[i * 4 + 0];
			}
			if (block[i * 4 + 1] < minNormal[1]) {
				minNormal[1] = block[i * 4 + 1];
			}
			if (block[i * 4 + 0] > maxNormal[0]) {
				maxNormal[0] = block[i * 4 + 0];
			}
			if (block[i * 4 + 1] > maxNormal[1]) {
				maxNormal[1] = block[i * 4 + 1];
			}
		}
	}

	void InsetNormalsBBoxDXT5(uint8_t *minNormal, uint8_t *maxNormal) {
		int inset[4];
		int mini[4];
		int maxi[4];

		inset[0] = (maxNormal[0] - minNormal[0]) - ((1 << (INSET_ALPHA_SHIFT - 1)) - 1);
		inset[1] = (maxNormal[1] - minNormal[1]) - ((1 << (INSET_COLOR_SHIFT - 1)) - 1);

		mini[0] = ((minNormal[0] << INSET_ALPHA_SHIFT) + inset[0]) >> INSET_ALPHA_SHIFT;
		mini[1] = ((minNormal[1] << INSET_COLOR_SHIFT) + inset[1]) >> INSET_COLOR_SHIFT;

		maxi[0] = ((maxNormal[0] << INSET_ALPHA_SHIFT) - inset[0]) >> INSET_ALPHA_SHIFT;
		maxi[1] = ((maxNormal[1] << INSET_COLOR_SHIFT) - inset[1]) >> INSET_COLOR_SHIFT;

		mini[0] = (mini[0] >= 0) ? mini[0] : 0;
		mini[1] = (mini[1] >= 0) ? mini[1] : 0;

		maxi[0] = (maxi[0] <= 255) ? maxi[0] : 255;
		maxi[1] = (maxi[1] <= 255) ? maxi[1] : 255;

		minNormal[0] = mini[0];
		minNormal[1] = (mini[1] & C565_6_MASK) | (mini[1] >> 6);

		maxNormal[0] = maxi[0];
		maxNormal[1] = (maxi[1] & C565_6_MASK) | (maxi[1] >> 6);
	}

	void InsetNormalsBBox3Dc(uint8_t *minNormal, uint8_t *maxNormal) {
		int inset[4];
		int mini[4];
		int maxi[4];

		inset[0] = (maxNormal[0] - minNormal[0]) - ((1 << (INSET_ALPHA_SHIFT - 1)) - 1);
		inset[1] = (maxNormal[1] - minNormal[1]) - ((1 << (INSET_ALPHA_SHIFT - 1)) - 1);

		mini[0] = ((minNormal[0] << INSET_ALPHA_SHIFT) + inset[0]) >> INSET_ALPHA_SHIFT;
		mini[1] = ((minNormal[1] << INSET_ALPHA_SHIFT) + inset[1]) >> INSET_ALPHA_SHIFT;

		maxi[0] = ((maxNormal[0] << INSET_ALPHA_SHIFT) - inset[0]) >> INSET_ALPHA_SHIFT;
		maxi[1] = ((maxNormal[1] << INSET_ALPHA_SHIFT) - inset[1]) >> INSET_ALPHA_SHIFT;

		mini[0] = (mini[0] >= 0) ? mini[0] : 0;
		mini[1] = (mini[1] >= 0) ? mini[1] : 0;

		maxi[0] = (maxi[0] <= 255) ? maxi[0] : 255;
		maxi[1] = (maxi[1] <= 255) ? maxi[1] : 255;

		minNormal[0] = mini[0];
		minNormal[1] = mini[1];

		maxNormal[0] = maxi[0];
		maxNormal[1] = maxi[1];
	}

	uint16_t NormalYTo565(uint8_t y) {
		return (static_cast<uint16_t>(y >> 2) << 5);
	}

	union OutData {
		uint8_t *bytes;
		uint16_t *words;
		uint32_t *dwords;

		OutData(uint8_t *data) : bytes(data) {}

		void b(uint8_t byte) {
			*bytes++ = byte;
		}

		void w(uint16_t word) {
			*words++ = word;
		}

		void d(uint32_t dword) {
			*dwords++ = dword;
		}

		void EmitGreenIndices(const uint8_t *block, const int offset, const uint8_t minGreen, const uint8_t maxGreen) {
			uint8_t mid = (maxGreen - minGreen) / (2 * 3);

			uint8_t gb1 = maxGreen - mid;
			uint8_t gb2 = (2 * maxGreen + 1 * minGreen) / 3 - mid;
			uint8_t gb3 = (1 * maxGreen + 2 * minGreen) / 3 - mid;

			block += offset;

			uint16_t result = 0;
			for (int i = 15; i >= 0; i--) {
				result <<= 2;
				uint8_t g = block[i * 4];
				int b1 = (g >= gb1);
				int b2 = (g >= gb2);
				int b3 = (g >= gb3);
				int index = (4 - b1 - b2 - b3) & 3;
				index ^= (2 > index);
				result |= index;
			}

			w(result);
		}

		void EmitAlphaIndices(const uint8_t *block, const int offset, const uint8_t minAlpha, const uint8_t maxAlpha) {
			uint8_t mid = (maxAlpha - minAlpha) / (2 * 7);

			uint8_t ab1 = maxAlpha - mid;
			uint8_t ab2 = (6 * maxAlpha + 1 * minAlpha) / 7 - mid;
			uint8_t ab3 = (5 * maxAlpha + 2 * minAlpha) / 7 - mid;
			uint8_t ab4 = (4 * maxAlpha + 3 * minAlpha) / 7 - mid;
			uint8_t ab5 = (3 * maxAlpha + 4 * minAlpha) / 7 - mid;
			uint8_t ab6 = (2 * maxAlpha + 5 * minAlpha) / 7 - mid;
			uint8_t ab7 = (1 * maxAlpha + 6 * minAlpha) / 7 - mid;

			block += offset;

			uint8_t indices[16];
			for (int i = 0; i < 16; i++) {
				uint8_t a = block[i * 4];
				int b1 = (a >= ab1);
				int b2 = (a >= ab2);
				int b3 = (a >= ab3);
				int b4 = (a >= ab4);
				int b5 = (a >= ab5);
				int b6 = (a >= ab6);
				int b7 = (a >= ab7);
				int index = (8 - b1 - b2 - b3 - b4 - b5 - b6 - b7) & 7;
				indices[i] = index ^ (2 > index);
			}

			b((indices[0] >> 0) | (indices[1] << 3) | (indices[2] << 6));
			b((indices[2] >> 2) | (indices[3] << 1) | (indices[4] << 4) | (indices[5] << 7));
			b((indices[5] >> 1) | (indices[6] << 2) | (indices[7] << 5));
			
			b((indices[8] >> 0) | (indices[9] << 3) | (indices[10] << 6));
			b((indices[10] >> 2) | (indices[11] << 1) | (indices[12] << 4) | (indices[13] << 7));
			b((indices[13] >> 1) | (indices[14] << 2) | (indices[15] << 5));
		}
	};
}

void CompressNormalMapDXT5(const uint8_t *inBuf, uint8_t *outBuf, int width, int height, int &outputBytes) {
	uint8_t block[64];
	uint8_t normalMin[4];
	uint8_t normalMax[4];

	OutData out(outBuf);

	for (int j = 0; j < height; j += 4, inBuf += width * 4 * 4) {
		for (int i = 0; i < width; i += 4) {

			ExtractBlock(inBuf + i * 4, width, block);

			GetMinMaxNormalsBBox(block, normalMin, normalMax);
			InsetNormalsBBoxDXT5(normalMin, normalMax);

			// Write out Nx into alpha channel.
			out.b(normalMax[0]);
			out.b(normalMin[0]);
			out.EmitAlphaIndices(block, 0, normalMin[0], normalMax[0]);

			// Write out Ny into green channel.
			out.d(NormalYTo565(normalMax[1]));
			out.d(NormalYTo565(normalMin[1]));
			out.EmitGreenIndices(block, 1, normalMin[1], normalMax[1]);
		}
	}

	outputBytes = out.bytes - outBuf;
}

void CompressNormalMap3Dc(const uint8_t *inBuf, uint8_t *outBuf, int width, int height, int &outputBytes) {
	uint8_t block[64];
	uint8_t normalMin[4];
	uint8_t normalMax[4];

	OutData out(outBuf);

	for (int j = 0; j < height; j += 4, inBuf += width * 4 * 4) {
		for (int i = 0; i < width; i += 4) {

			ExtractBlock(inBuf + i * 4, width, block);

			GetMinMaxNormalsBBox(block, normalMin, normalMax);
			InsetNormalsBBox3Dc(normalMin, normalMax);

			// Write out Nx as an alpha channel.
			out.b(normalMax[0]);
			out.b(normalMin[0]);
			out.EmitAlphaIndices(block, 0, normalMin[0], normalMax[0]);

			// Write out Ny as an alpha channel.
			out.b(normalMax[1]);
			out.b(normalMin[1]);
			out.EmitAlphaIndices(block, 1, normalMin[1], normalMax[1]);
		}
	}

	outputBytes = out.bytes - outBuf;
}