#ifndef MOTU_Z_AXIS_COLLIDER
#define MOTU_Z_AXIS_COLLIDER

#include <vector>
#include <memory>
#include "bounding_rectangle.h"
#include "raster.h"

namespace motu {
	struct Mesh;
	struct Vector2;
	struct Raster;

	class ZAxisCollider {
	public:
		ZAxisCollider(const Mesh &mesh) : mesh(mesh){
			loadLookupTable();
		}

		//tri is the offset off the hit triangle being -1 in the case of a complete miss;
		float heightAt(Vector2 pos, int &tri) const;

		std::unique_ptr<Raster> drawTable() const;

	private:
		const Mesh &mesh;
		BoundingRectangle bounds;
		std::vector<std::pair<int, int>> lookupTable;
		std::vector<int> data;
		int tableSize, cellSize;

		void loadLookupTable();
	};
}

#endif