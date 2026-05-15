
#include "Vortrix/Maths/VortrixMaths.h"
#include "AABB.h"



namespace vx {
	class OBB
	{
	public:
		OBB(const Quat& orientation, const Vec3& center, const Vec3& half_extent) :
			mOrientation(orientation), mCenter(center), mHalfExtent(half_extent) {
		}

		OBB(const AABB& aabb, const Quat& orientation) :
			mOrientation(orientation), mCenter(aabb.GetCenter()), mHalfExtent(aabb.GetHalfExtent())
		{
		}

		const Quat& Orientation() const { return mOrientation; }
		const Vec3& Center() const { return mCenter; }
		const Vec3& HalfExtent() const { return mHalfExtent; }

		std::array<Vec3, 8> ComputeCorners(const Vec3& translate) const
		{
			Vec3 center = mOrientation.Rotate(mCenter) + translate;

			Vec3 a0 = mOrientation.RotateAxisX() * mHalfExtent.X();
			Vec3 a1 = mOrientation.RotateAxisY() * mHalfExtent.Y();
			Vec3 a2 = mOrientation.RotateAxisZ() * mHalfExtent.Z();

			return{
			center - a0 - a1 - a2,
			center + a0 - a1 - a2,
			center + a0 + a1 - a2,
			center - a0 + a1 - a2,
			center - a0 - a1 + a2,
			center + a0 - a1 + a2,
			center + a0 + a1 + a2,
			center - a0 + a1 + a2 };
		}
	private:
		Quat mOrientation;
		//bounds center 
		Vec3 mCenter;
		Vec3 mHalfExtent;
	};

}