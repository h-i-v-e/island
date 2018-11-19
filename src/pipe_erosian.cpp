#include "pipe_erosian.h"
#include "height_map.h"
#include "vector2int.h"

#include <algorithm>

using namespace motu;

namespace {

	struct Eroder{

		struct Flow {
			float right, down, outflow, invOutflow;

			Flow() : right(0.0f), down(0.0f), outflow(0.0f), invOutflow(0.0f) {}
		};

		struct State {
			float silt, water;

			State() : silt(0.0f), water(0.0f) {}
		};

		std::default_random_engine &rd;
		HeightMap &heightMap;
		Grid<State> state;
		Grid<Flow> flow;
		Grid<State> scratch;
		std::vector<float> noise;

		Eroder(std::default_random_engine &rd, HeightMap &hm) :
			rd(rd), heightMap(hm), state(hm.width(), hm.height()),
			flow(hm.width(), hm.height()), scratch(hm.width(), hm.height()),
			noise(hm.width() * (hm.height() << 1)) {
			std::uniform_real_distribution<float> rain(0.0f, 0.1f / hm.width());
			for (auto i = noise.begin(); i != noise.end(); ++i) {
				*i = rain(rd);
			}
		}

		void addRain() {
			std::uniform_int_distribution<size_t> offset(0, heightMap.length());
			float *n = noise.data() + offset(rd);
			for (auto *i = state.data(), *j = state.data() + state.length(); i != j; ++i) {
				i->water += *n++;
			}
		}

		void calculateFlow() {
			for (int y = 1; y != heightMap.height() - 1; ++y) {
				for (int x = 1; x != heightMap.width() - 1; ++x) {
					float water = state(x, y).water;
					if (water <= 0.0f) {
						continue;
					}
					float height = heightMap(x, y);
					if (height < 0.0f) {
						state(x, y).water = water = 0.0f;
						if (heightMap(x - 1, y) < 0.0f && heightMap(x + 1, y) < 0.0f && heightMap(x, y - 1) < 0.0f && heightMap(x, y + 1) < 0.0f) {
							continue;
						}
					}
					Flow &f = flow(x, y);
					float surface = height + water;
					f.outflow = 0.0f;
					f.right = surface - (state(x + 1, y).water + heightMap(x + 1, y));
					if (f.right > 0.0f) {
						f.outflow += f.right;
					}
					f.down = surface - (state(x, y + 1).water + heightMap(x, y + 1));
					if (f.down > 0.0f) {
						f.outflow += f.down;
					}
					float old = flow(x - 1, y).right;
					if (old < 0.0f) {
						f.outflow -= old;
					}
					old = flow(x, y - 1).down;
					if (old < 0.0f) {
						f.outflow -= old;
					}
					f.invOutflow = f.outflow < FLT_EPSILON ? 0.0f :  1.0f / f.outflow;
				}
			}
		}

		void gatherSilt() {
			for (int y = 1; y != heightMap.height() - 1; ++y) {
				for (int x = 1; x != heightMap.width() - 1; ++x) {
					State &s = state(x, y);
					if (s.water > 0.0f || s.silt > 0.0f) {
						Flow &f = flow(x, y);
						float silt = s.water * f.outflow;
						heightMap(x, y) += (s.silt - silt);
						s.silt = silt;
					}
				}	
			}
		}

		void moveWater() {
			std::fill(scratch.data(), scratch.data() + scratch.length(), State());
			for (int y = 1; y != heightMap.height() - 1; ++y) {
				for (int x = 1; x != heightMap.width() - 1; ++x) {
					const Flow &f = flow(x, y);
					if (f.right > 0.0f) {
						State &in = state(x, y);
						State &out = scratch(x + 1, y);
						float weight = f.right * f.invOutflow;
						out.water += in.water * weight;
						out.silt += in.silt * weight;
					}
					else if (f.right < 0.0f){
						State &in = state(x + 1, y);
						State &out = scratch(x, y);
						float weight = f.right * - flow(x + 1, y).invOutflow;
						out.water += in.water * weight;
						out.silt += in.silt * weight;
					}
					if (f.down > 0.0f) {
						State &in = state(x, y);
						State &out = scratch(x, y + 1);
						float weight = f.down * f.invOutflow;
						out.water += in.water * weight;
						out.silt += in.silt * weight;
					}
					else if (f.down < 0.0f) {
						State &in = state(x, y + 1);
						State &out = scratch(x, y);
						float weight = f.down * - flow(x, y + 1).invOutflow;
						out.water += in.water * weight;
						out.silt += in.silt * weight;
					}
				}
			}
			std::copy(scratch.data(), scratch.data() + scratch.length(), state.data());
		}

		void cleanup() {
			for (int y = 1; y != heightMap.height() - 1; ++y) {
				for (int x = 1; x != heightMap.width() - 1; ++x) {
					heightMap(x, y) += state(x, y).silt;
				}
			}
		}

		void erode(int steps) {
			for (int i = 0; i != steps; ++i) {
				addRain();
				for (int i = 0; i != steps; ++i) {
					calculateFlow();
					gatherSilt();
					moveWater();
				}
			}
			//cleanup();
		}
	};
}

void PipeErosian::erode() {
	Eroder eroder(rd, heightMap);
	eroder.erode(6);
}