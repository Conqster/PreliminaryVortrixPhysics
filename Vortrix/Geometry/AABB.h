#pragma once


#include "Vortrix/Maths/Vec3.h"
#include "Vortrix/Maths/Mat44.h"

#include <array>


namespace vx
{



	struct AABB
	{
	public:
		//Constructor
		AABB() : mMin(Vec3(0.0f)), mMax(Vec3(0.0f)) {}
		AABB(const float half_size) : mMin(Vec3(-half_size)), mMax(Vec3(half_size)) {}
		AABB(const Vec3& min, const Vec3& max) : mMin(min), mMax(max) {}
		AABB(const Vec3& pointA) : mMin(pointA), mMax(pointA) {} //used if only a point is defines

		static AABB FromPoints(const Vec3& pt0, const Vec3& pt1)
		{
			return AABB(Vec3::Min(pt0, pt1), Vec3::Max(pt0, pt1));
		}

		Vec3 GetCenter() const
		{
			//return 0.5f * (min + max);
			return (mMin + mMax) * 0.5f;
		}

		Vec3 GetSize() const
		{
			return mMax - mMin;
		}

		float GetVolume() const
		{
			Vec3 extent = mMax - mMin;
			return extent.X() * extent.Y() * extent.Z();
		}


		/// 2(xy + yz + xz)
		float GetSurfaceArea() const
		{
			Vec3 extent = mMax - mMin;
			return 2.0f * (extent.X() * extent.Y() +
				extent.Y() * extent.Z() +
				extent.X() * extent.Z());
		}

		Vec3 GetHalfExtent() const
		{
			return (mMax - mMin) * 0.5f;
		}

		bool IsValid() const {
			return mMin.X() <= mMax.X() &&
				mMin.Y() <= mMax.Y() &&
				mMin.Z() <= mMax.Z();
		}

		void Translate(const Vec3& translation)
		{
			mMin += translation;
			mMax += translation;
		}


		void Expand(const Vec3& scale)
		{
			mMin -= scale;
			mMax += scale;
		}
		void Grow(const float size)
		{
			mMin -= Vec3(size);
			mMax += Vec3(size);
		}

		void Merge(const Vec3& pointA)
		{
			mMin = Vec3::Min(mMin, pointA);
			mMax = Vec3::Max(mMax, pointA);
		}

		void Merge(const AABB& rhs)
		{
			mMin = Vec3::Min(mMin, rhs.mMin);
			mMax = Vec3::Max(mMax, rhs.mMax);
			VX_ASSERT_WARN(IsValid(), "AABB became invalid after merge");
		}

		AABB Merged(const AABB& rhs) const
		{
			Vec3 _min = Vec3::Min(mMin, rhs.mMin);
			Vec3 _max = Vec3::Max(mMax, rhs.mMax);
			VX_ASSERT_WARN(IsValid(), "AABB became invalid after merge");

			return AABB(_min, _max);
		}

		void Reset()
		{
			mMin = Vec3(kMaxf);
			mMax = Vec3(-kMaxf);
		}

		void Transform(const Mat44& transform)
		{
			AABB result = Transformed(transform);
			*this = result;
		}

		AABB Transformed(const Mat44& transform) const
		{
			//New AABB center to be in tranform pos
			Vec3 temp_min = transform.GetTranslation();
			Vec3 temp_max = temp_min;


			for (uint i = 0; i < 3; i++)
			{
				Vec3 column = transform.GetColumn(i);

				Vec3 a = column * mMin[i];
				Vec3 b = column * mMax[i];

				temp_min += Vec3::Min(a, b);
				temp_max += Vec3::Max(a, b);
			}

			return AABB(temp_min, temp_max);
		}


		AABB Scaled(const Vec3& scale) const
		{
			return AABB::FromPoints(mMin * scale, mMax * scale);
		}

		//Check if this box overlaps with another
		bool Overlaps(const AABB& rhs) const
		{
			/// minX <= rhs.maxX && maxX >= rhs.minX
			/// minY <= rhs.maxY && maxY >= rhs.minY
			/// minZ <= rhs.maxZ && maxZ >= rhs.minZ
			/// 
			/// 
			return Vec3::LessOrEq(mMin, rhs.mMax) && Vec3::GreaterOrEq(mMax, rhs.mMin);
			//return (mMin.X() <= rhs.mMax.X() + kEpsilon && mMax.X() + kEpsilon >= rhs.mMin.X()) &&
			//	(mMin.Y() <= rhs.mMax.Y() + kEpsilon && mMax.Y() + kEpsilon >= rhs.mMin.Y()) &&
			//	(mMin.Z() <= rhs.mMax.Z() + kEpsilon && mMax.Z() + kEpsilon >= rhs.mMin.Z());
		}

		bool Contains(const Vec3& pointA) const
		{
			return pointA.X() >= mMin.X() && pointA.X() <= mMax.X() &&
					pointA.Y() >= mMin.Y() && pointA.Y() <= mMax.Y() &&
					pointA.Z() >= mMin.Z() && pointA.Z() <= mMax.Z();
		}

		bool Contains(const AABB& rhs) const
		{
			return rhs.mMin.X() >= mMin.X() && rhs.mMax.X() <= mMax.X() &&
			rhs.mMin.Y() >= mMin.Y() && rhs.mMax.Y() <= mMax.Y() &&
				rhs.mMin.Z() >= mMin.Z() && rhs.mMax.Z() <= mMax.Z();
		}



		std::array<Vec3, 8> GetCorners() const
		{
			return {
			   Vec3(mMin.X(), mMin.Y(), mMin.Z()),
			   Vec3(mMax.X(), mMin.Y(), mMin.Z()),
			   Vec3(mMax.X(), mMax.Y(), mMin.Z()),
			   Vec3(mMin.X(), mMax.Y(), mMin.Z()),
			   Vec3(mMin.X(), mMin.Y(), mMax.Z()),
			   Vec3(mMax.X(), mMin.Y(), mMax.Z()),
			   Vec3(mMax.X(), mMax.Y(), mMax.Z()),
			   Vec3(mMin.X(), mMax.Y(), mMax.Z())
			};
		}

		static std::array<Vec3, 8> GetCorners(const AABB& aabb) { return aabb.GetCorners(); }

		Vec3 mMin = Vec3(0.0f);
		Vec3 mMax = Vec3(0.0f);



		//std::array<Vec3, 8> GetOBB(const Mat44& transform)
		//{
		//	Vec3 localCenter = (mMin + mMax) * 0.5f;
		//	Vec3 halfExtents = (mMax - mMin) * 0.5f;

		//	// World center
		//	Vec3 center = transform.Transform(localCenter);


		//	Vec3 a0 = transform.GetAxisX() * halfExtents.X();
		//	Vec3 a1 = transform.GetAxisY() * halfExtents.Y();
		//	Vec3 a2 = transform.GetAxisZ() * halfExtents.Z();

		//	return{
		//	center - a0 - a1 - a2,
		//	center + a0 - a1 - a2,
		//	center + a0 + a1 - a2,
		//	center - a0 + a1 - a2,
		//	center - a0 - a1 + a2,
		//	center + a0 - a1 + a2,
		//	center + a0 + a1 + a2,
		//	center - a0 + a1 + a2 };
		//}


		//std::array<Vec3, 8> GetOBB(const Quat& q, const Vec3& t)
		//{
		//	Vec3 localCenter = (mMin + mMax) * 0.5f;
		//	Vec3 halfExtents = (mMax - mMin) * 0.5f;

		//	Vec3 center = q.Rotate(localCenter) + t;


		//	Vec3 a0 = q.RotateAxisX() * halfExtents.X();
		//	Vec3 a1 = q.RotateAxisY() * halfExtents.Y();
		//	Vec3 a2 = q.RotateAxisZ() * halfExtents.Z();

		//	return{
		//	center - a0 - a1 - a2,
		//	center + a0 - a1 - a2,
		//	center + a0 + a1 - a2,
		//	center - a0 + a1 - a2,
		//	center - a0 - a1 + a2,
		//	center + a0 - a1 + a2,
		//	center + a0 + a1 + a2,
		//	center - a0 + a1 + a2 };
		//}



	};
}