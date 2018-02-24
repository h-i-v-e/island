#ifndef motu_colour_h
#define motu_colour_h

#include <cstdint>

#include "vector3.h"

namespace motu {
	template <class IntType>
	IntType gunToUint(float val, float multiplier) {
		float out = multiplier * val;
		if (out < 0.0f) {
			return 0;
		}
		if (out > multiplier) {
			out = multiplier;
		}
		return static_cast<IntType>(out);
	}
	
	inline uint32_t toColour32(const Vector3 &vec) {
		return (gunToUint<uint32_t>(vec.x, 255.0f) << 16) | (gunToUint<uint32_t>(vec.y, 255.0f) << 8) | gunToUint<uint32_t>(vec.z, 255.0f);
	}

	inline uint16_t toColour16(const Vector3 &vec) {
		return (gunToUint<uint16_t>(vec.x, 63.0f) << 11) | (gunToUint<uint16_t>(vec.y, 127.0f) << 5) | gunToUint<uint16_t>(vec.z, 63.0f);
	}

	inline uint16_t toColour16(uint32_t colour) {
		uint16_t red = (colour >> 19) & 0x1f, green = (colour >> 10) & 0x3f, blue = (colour >> 3) & 0x1f;
		return (red << 11) | (green << 5) | blue;
	}

	inline uint32_t toColour32(uint16_t colour) {
		uint32_t red = (colour >> 11) & 0x1f, green = (colour >> 5) & 0x3f, blue = colour & 0x1f;
		return (red << 19) | (green << 10) | (blue << 3);
	}

	inline Vector3 toColourV3(uint32_t colour) {
		static float mul = 1.0f / 255.0f;
		return Vector3((colour >> 16) * mul, ((colour >> 8) & 0xff00) * mul, (colour & 0xff) * mul);
	}

	inline Vector3 toColourV3(uint16_t colour) {
		static float mulA = 1.0f / 63.0f, mulB = 1.0f / 127.0f;
		return Vector3((colour >> 11) * mulA, ((colour >> 5) & 0x3f) * mulB, (colour & 0x1f) * mulA);
	}
}

#endif