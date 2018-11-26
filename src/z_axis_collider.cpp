#include "z_axis_collider.h"
#include "mesh.h"
#include "triangle.h"
#include "bounding_rectangle.h"
#include "vector2int.h"
#include "triangle3.h"
#include <cmath>
#include <algorithm>
#include "raster.h"
#include "bounding_box.h"

using namespace motu;

namespace {
	inline void makeBoundingRectangle(const Mesh &mesh, BoundingRectangle &rect) {
		for (const Vector3 &vec : mesh.vertices) {
			rect.add(vec.asVector2());
		}
		float diff = rect.width() - rect.height();
		if (diff > 0.0f) {
			rect.bottomRight.x += diff;
		}
		else if (diff < 0.0f) {
			rect.bottomRight.y -= diff;
		}
	}

	Vector2Int getOffset(const BoundingRectangle &rect, size_t dim, Vector2 pos) {
		pos -= rect.topLeft;
		pos *= dim;
		return Vector2Int(pos.x, pos.y);
	}

	struct Indexer {
		size_t dim;
		float invDim;
		BoundingRectangle rect;

		Indexer(const Mesh &mesh) {
			dim = static_cast<size_t>(sqrt(static_cast<double>(mesh.triangles.size())));
			invDim = 1.0f / static_cast<float>(dim);
			makeBoundingRectangle(mesh, rect);
		}

		template<class Assign>
		void withTriangles(const Mesh &mesh, Assign assign) {
			for (size_t i = 0, j = mesh.triangles.size(); i != j; i += 3) {
				BoundingRectangle rect;
				rect.add(mesh.vertices[mesh.triangles[i]].asVector2());
				rect.add(mesh.vertices[mesh.triangles[i + 1]].asVector2());
				rect.add(mesh.vertices[mesh.triangles[i + 2]].asVector2());
				Vector2Int topLeft(getOffset(this->rect, dim, rect.topLeft));
				Vector2Int bottomRight(getOffset(this->rect, dim, rect.bottomRight));
				while (topLeft.y <= bottomRight.y) {
					int yOff = topLeft.y * dim;
					for (int x = topLeft.x; x <= bottomRight.x; ++x) {
						assign(yOff + x, i);
					}
					++topLeft.y;
				}
			}
		}

		struct CellSizeAssign {
			std::vector<std::pair<int, int>> *buffer;

			CellSizeAssign(std::vector<std::pair<int, int>> &buf) : buffer(&buf) {}

			void operator()(int offset, int index) {
				++(*buffer)[offset].second;
			}
		};

		void computeCellSizes(const Mesh &mesh, std::vector<std::pair<int, int>> &buffer) {
			withTriangles(mesh, CellSizeAssign(buffer));
			int offset = 0;
			for (auto &i : buffer) {
				i.first = offset;
				offset += i.second;
				i.second = 0;
			}
		}
	};

	struct LookupTableAssign {
		std::vector<std::pair<int, int>> *table;
		std::vector<int> *buffer;

		LookupTableAssign(std::vector<std::pair<int, int>> &table, std::vector<int> &buffer) : table(&table), buffer(&buffer){
		}

		void operator()(int position, int index) {
			auto i = (*table)[position];
			(*buffer)[i.first + i.second++] = index;
		}
	};
}

void ZAxisCollider::loadLookupTable() {
	Indexer indexer(mesh);
	tableSize = indexer.dim;
	bounds = indexer.rect;
	lookupTable.resize(tableSize * tableSize);
	indexer.computeCellSizes(mesh, lookupTable);
	data.resize(lookupTable.back().first + lookupTable.back().second);
	indexer.withTriangles(mesh, LookupTableAssign(lookupTable, data));
}

std::unique_ptr<Raster> ZAxisCollider::drawTable() const {
	//logTable(lookupTable, tableSize, cellSize);
	auto ret = std::make_unique<Raster>(tableSize, tableSize);
	auto data = ret->data();
	for (size_t i = 0, j = lookupTable.size(); i != j; i += cellSize) {
		*data++ = lookupTable[i].second == 0 ? 0x00000000 : 0xffffffff;
	}
	return std::move(ret);
}

float ZAxisCollider::heightAt(Vector2 v2, int &tri) const{
	tri = -1;
	if (!bounds.contains(v2)) {
		return std::numeric_limits<float>::min();
	}
	Vector2Int pos = getOffset(bounds, tableSize, v2);
	float max = std::numeric_limits<float>::min();
	Spline test(Vector3(v2.x, v2.y, 1.0f), Vector3(v2.x, v2.y, -1.0f));
	auto entry = lookupTable[pos.y * tableSize + pos.x];
	for (int end = entry.first + entry.second; entry.first != end; ++entry.first) {
		Triangle3 t3 = mesh.triangle(data[entry.first]);
		Vector3 hit;
		if (t3.intersection(test, hit) && hit.z > max) {
			max = hit.z;
			tri = entry.first;
		}
	}
	return max;
}