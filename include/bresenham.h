#include <algorithm>

namespace motu {
	//line algorithm
	template<class Setter>
	class Bresenham {
	public:
		Bresenham(Setter setter) : setter(setter) {}

		void operator()(int ax, int ay, int bx, int by) {
			int octant = GetOctant(ax, bx, ay, by);
			SwitchToOctantZero(octant, ax, ay);
			SwitchToOctantZero(octant, bx, by);
			int dx = bx - ax, dy = by - ay;
			int twiceDx = dx << 1, twiceDy = dy << 1;
			int D = twiceDy - dx, y = ay;
			for (int x = ax; x <= bx; ++x) {
				int ox = x, oy = y;
				SwitchFromOctantZero(octant, ox, oy);
				setter(ox, oy);
				if (D > 0) {
					++y;
					D -= twiceDx;
				}
				D += twiceDy;
			}
		}

		Setter setter;
	private:

		int GetOctant(int x1, int x2, int y1, int y2) {
			if (x1 > x2) {
				if (y1 > y2) {
					return x1 - x2 > y1 - y2 ? 4 : 5;
				}
				else {
					return x1 - x2 > y2 - y1 ? 3 : 2;
				}
			}
			else if (y1 > y2) {
				return x2 - x1 > y1 - y2 ? 7 : 6;
			}
			else {
				return x2 - x1 > y2 - y1 ? 0 : 1;
			}
		}

		void SwitchToOctantZero(int octant, int &x, int &y) {
			switch (octant) {
			case 1:
				std::swap(x, y);
				break;
			case 2:
				std::swap(x, y);
				y = -y;
				break;
			case 3:
				x = -x;
				break;
			case 4:
				x = -x;
				y = -y;
				break;
			case 5:
				std::swap(x, y);
				x = -x;
				y = -y;
				break;
			case 6:
				std::swap(x, y);
				x = -x;
				break;
			case 7:
				y = -y;
				break;
			}
		}

		void SwitchFromOctantZero(int octant, int &x, int &y) {
			switch (octant) {
			case 1:
				std::swap(x, y);
				break;
			case 2:
				std::swap(x, y);
				x = -x;
				break;
			case 3:
				x = -x;
				break;
			case 4:
				x = -x;
				y = -y;
				break;
			case 5:
				std::swap(x, y);
				x = -x;
				y = -y;
				break;
			case 6:
				std::swap(x, y);
				y = -y;
				break;
			case 7:
				y = -y;
				break;
			default:
				break;
			}
		}
	};
}