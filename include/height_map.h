#include "grid.h"

#ifndef MOTU_HEIGHT_MAP
#define MOTU_HEIGHT_MAP

namespace motu {
	struct Mesh;
	struct Vector3;

	class HeightMap : public Grid<float> {
	public:
		HeightMap(int width, int height) : Grid(width, height){}

		void load(const Mesh &mesh, float maxHeight);

		void load(const Mesh &mesh);

		void smooth();

		float interpTLBR(float x, float y);

		float interpTRBL(float x, float y);

		void raise(const Vector3 &);

		float seaLevel() const {
			return mSeaLevel;
		}
	private:
		float mSeaLevel;
	};
}
#endif