#include "grid.h"
#include "vector2int.h"

#ifndef MOTU_HEIGHT_MAP
#define MOTU_HEIGHT_MAP

namespace motu {
	struct Mesh;
	struct Vector3;

	class HeightMap : public Grid<float> {
	public:
		HeightMap(int width, int height) : Grid(width, height){}

		void assignNeighbourBounds(int x, int y, Vector2Int &topLeft, Vector2Int &bottomRight) {
			assign(x, topLeft.x, bottomRight.x, width());
			assign(y, topLeft.y, bottomRight.y, height());
		}

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

		void assign(int val, int &from, int &to, int max) {
			from = val == 0 ? 0 : val - 1;
			to = val + 1;
			if (to == max) {
				--to;
			}
		}
	};
}
#endif