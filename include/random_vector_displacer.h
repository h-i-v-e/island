#ifndef MOTU_RANDOM_VECTOR_DISPLACER
#define MOTU_RANDOM_VECTOR_DISPLACER

#include "mesh.h"

#include <random>

namespace motu {
	class RandomVectorDisplacer {
	public:
		RandomVectorDisplacer() : randomTwoPi(0.0f, std::acosf(-1) * 2.0f) {}

		Vector2 randomDirection(std::default_random_engine &rd) const{
			float angle = randomTwoPi(rd);
			return Vector2(std::cosf(angle), std::sinf(angle));
		}

		Vector2 createDisplacement(std::default_random_engine &rd, const Mesh &mesh, const MeshEdgeMap &mep, int idx, float mul = 0.5f) const{
			float min = std::numeric_limits<float>::max();
			const Vector2 &pos = mesh.vertices[idx].asVector2();
			for (auto opts = mep.vertex(idx); opts.first != opts.second; ++opts.first) {
				float sqrDist = (mesh.vertices[*opts.first].asVector2() - pos).sqrMagnitude();
				if (sqrDist < min) {
					min = sqrDist;
				}
			}
			return pos + randomDirection(rd) * std::uniform_real_distribution<float>(0.0f, sqrtf(min) * mul)(rd);
		}
	private:
		std::uniform_real_distribution<float> randomTwoPi;
	};
}

#endif