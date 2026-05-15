#pragma once

#include "Maths/Vec3.h"

#include <array>


namespace vx{
	class Body;

	struct ManifoldPoint
	{
		static bool kUseNewManifoldPt;

		/// points in Manifold world space
		Vec3 pointA = Vec3(0.0f);
		float peneration = 0.0f;
		Vec3 pointB = Vec3(0.0f);
	};


	struct ContactManifold
	{
		Body* a = nullptr;
		Body* b = nullptr;

		Vec3 normal = Vec3::Up();

		//std::vector<ManifoldPoint> points;
		const static int kMaxPoints = 4;
		std::array<ManifoldPoint, kMaxPoints> points{};
		int numManifoldPoints = 0;


		void Store(const ManifoldPoint* _points, int count)
		{
			numManifoldPoints = VxMin(count, kMaxPoints);
			//std::memcpy(points.data(), _points, count * sizeof(ManifoldPoint)); //vec3 not trovia
			std::copy(_points,
				_points + numManifoldPoints,
				points.begin());
		}

	private:
		friend class ContactConstraintSolver;
		friend class CollisionDispatcher;
		//prevent external swap, except ContactConstraintSolver
		VX_INLINE void Swap()
		{
			std::swap(a, b);
			Flip();
		}
		VX_INLINE void Flip()
		{
			normal = -normal;

			for (int i = 0; i < numManifoldPoints; ++i)
			{
				auto& mp = points[i];
				std::swap(mp.pointA, mp.pointB);
			}
		}
	};
}