#include "grid.h"

namespace motu {
	struct Mesh;

	class HeightMap : public Grid<float> {
	public:
		HeightMap(int width, int height) : Grid(width, height){}

		void load(const Mesh &mesh, float maxHeight);

		void load(const Mesh &mesh);

		float seaLevel() const {
			return mSeaLevel;
		}
	private:
		float mSeaLevel;
	};
}