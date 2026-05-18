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
		static constexpr int kMaxPoints = 4;

		ContactManifold(Body* _a, Body* _b) :
			a(_a), b(_b), normal(Vec3::Up()),
			mPoints({}), mPointCount(0) {}

		~ContactManifold() = default;

		Body* a = nullptr;
		Body* b = nullptr;

		Vec3 normal = Vec3::Up();


		VX_INLINE int PointCount() const { return mPointCount; }


		/// use PointsPtr & SetPoint sparingly 
		/// inotder to support low-level manipulation 
		/// like geometrical sorting, then set count etc
		VX_INLINE ManifoldPoint* PointsPtr() { return mPoints.data(); }
		VX_INLINE const ManifoldPoint* Points() const { return mPoints.data(); }
		VX_INLINE void SetPointCount(int count)
		{ 
			VX_ASSERT(count <= kMaxPoints);
			mPointCount = count; 
		}

		VX_INLINE void Store(const ManifoldPoint* _points, int count)
		{
			mPointCount = VxMin(count, kMaxPoints);
			//std::memcpy(points.data(), _points, count * sizeof(ManifoldPoint)); //vec3 not trovia
			std::copy(_points,
				_points + mPointCount,
				mPoints.begin());
		}

		VX_INLINE void AddPoint(const Vec3& point_a, const Vec3& point_b, float penetration)
		{
			VX_ASSERT(mPointCount < kMaxPoints);

			ManifoldPoint& mp = mPoints[mPointCount++];
			mp.pointA = point_a;
			mp.peneration = penetration;
			mp.pointB = point_b;
		}

		VX_INLINE void Clear() { mPointCount = 0; }
	private:

		std::array<ManifoldPoint, kMaxPoints> mPoints{};
		int mPointCount = 0;

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

			for (int i = 0; i < mPointCount; ++i)
			{
				auto& mp = mPoints[i];
				std::swap(mp.pointA, mp.pointB);
			}
		}
	};
}