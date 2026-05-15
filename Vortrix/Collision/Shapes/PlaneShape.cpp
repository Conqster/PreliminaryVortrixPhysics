#include "PlaneShape.h"
#include "Geometry/AABB.h"

namespace vx {
	AABB PlaneShape::ComputeLocalBounds()
	{
		//tangent basis
		Vec3 t0 = mNormal.NormalisedPerpendicular();
		Vec3 t1 = t0.Cross(mNormal).Normalised();
		t0 = mNormal.Cross(t1);

		//scale
		t0 *= mHalfExtent;
		t1 *= mHalfExtent;

		Vec3 vertices[4];
		Vec3 pt = -mNormal * mConstant;
		vertices[0] = pt + t0 + t1;
		vertices[1] = pt + t0 - t1;
		vertices[2] = pt - t0 - t1;
		vertices[3] = pt - t0 + t1;

		mLocalBounds = AABB(vertices[0]);
		for (const Vec3& v : vertices)
		{
			mLocalBounds.Merge(v);
			mLocalBounds.Merge(v - mHalfExtent * mNormal);
			//mLocalBounds.Merge(v+mHalfExtent*mNormal);
		}
		return mLocalBounds;
	}
} //namespace vx