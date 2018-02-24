#ifndef MATRIX4_H
#define MATRIX4_H

#include "vector3.h"

namespace motu {
	struct Matrix4 {
		float data[4][4];

		struct Vector4 {
			float data[4];
		};

		static Vector4 asPoint(const Vector3 &vec) {
			return { vec.x, vec.y, vec.z, 1.0f };
		}

		static Vector4 asVector(const Vector3 &vec) {
			return { vec.x, vec.y, vec.z, 0.0f };
		}

		Vector4 operator*(const Vector4 &vec) const {
			Vector4 out;
			for (int i = 0; i != 4; ++i) {
				float total = 0.0f;
				for (int j = 0; j != 4; ++j) {
					total += data[i][j];
				}
				out.data[i] = total;
			}
			return out;
		}

		static Matrix4 identity() {
			return {
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
		}

		Matrix4 operator*(const Matrix4 &o) const {
			Matrix4 out;
			for (int i = 0; i != 4; ++i) {
				for (int j = 0; j != 4; ++j) {
					float total = 0.0f;
					for (int k = 0; k != 4; ++k) {
						total += data[i][k] * data[k][j];
					}
					out.data[i][j] = total;
				}
			}
			return out;
		}

		static Matrix4 scale(float amount) {
			return {
				amount, 0.0f, 0.0f, 0.0f,
				0.0f, amount, 0.0f, 0.0f,
				0.0f, 0.0f, amount, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			};
		}

		static Matrix4 translate(Vector3 amount) {
			return {
				1.0f, 0.0f, 0.0f, amount.x,
				0.0f, 1.0f, 0.0f, amount.y,
				0.0f, 0.0f, 1.0f, amount.z,
				0.0f, 0.0f, 0.0f, 1.0f
			};
		}
	};
}

#endif