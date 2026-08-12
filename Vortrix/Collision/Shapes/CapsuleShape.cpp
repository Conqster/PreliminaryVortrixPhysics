#include "CapsuleShape.h"
#include "Vortrix/Geometry/AABB.h"

namespace vx {

	AABB CapsuleShape::LocalBounds() const
	{
		Vec3 half_extent = HalfExtents();
		return AABB(-half_extent, half_extent);
	}
	AABB CapsuleShape::ComputeWorldBounds(const Mat44& tranform, const Vec3& scale) const
	{
		Vec3 scaled_half_extent = scale.Abs() * Vec3(mRadius, mCylinderHalfHeight, mRadius);

		Vec3 center = tranform.GetTranslation();
		Vec3 axis = tranform.Multiply3x3(GetLocalAxis());

		Vec3 p0 = center - axis * scaled_half_extent.Y();
		Vec3 p1 = center + axis * scaled_half_extent.Y();

		Vec3 rvec = scaled_half_extent.SplatX();

		Vec3 _min = Vec3::Min(p0, p1) - rvec;
		Vec3 _max = Vec3::Max(p0, p1) + rvec;

		return AABB(_min, _max);
	}
	Vec3 CapsuleShape::SupportWS(const Mat44& in_transform, Vec3 dir) const
	{
		const Vec3 local_dir = in_transform.Multiply3x3Transposed(dir);

		Vec3 pointA = local_dir.Y() > 0 ?
			Vec3(0.0f, mCylinderHalfHeight, 0.0f) :
			Vec3(0.0f, -mCylinderHalfHeight, 0.0f);

		const Vec3 local_pt = pointA + local_dir.Normalised() * mRadius;
		return in_transform.Multiply3x3(local_pt) + in_transform.GetTranslation();
	}
	MassProperties CapsuleShape::GetMassProperties() const
	{
		const float radius_sq = mRadius * mRadius;
		const float h = mCylinderHalfHeight * 2.0f;
		const float h_sq = h * h;

		float cylinder_mass = kVxPi * h * radius_sq * mDensity;
		float hemisphere_mass = (2.0f * kVxPi / 3.0f) * radius_sq * mRadius * mDensity;


		//cylinder
		float height_sq = VxSqr(h);
		float Iy = radius_sq * cylinder_mass * 0.5f;
		float Ix = Iy * 0.5f + cylinder_mass * height_sq / 12.0f;

		// From hemispheres
		const float temp = hemisphere_mass * 4.0f * radius_sq / 5.0f;
		Iy += temp * 2.0f;
		Ix += temp + hemisphere_mass * (0.5f * height_sq + (3.0f / 4.0f) * h * mRadius);


		const float Mtotal = cylinder_mass + hemisphere_mass * 2.0f;

		MassProperties mp;
		mp.mass = Mtotal;
		mp.inertialTensorDiagonal = Float3(Ix, Iy, Ix);
		return mp;
	}
	//MassProperties CapsuleShape::GetMassProperties() const
	//{
	//	const float r = mRadius;
	//	const float r2 = r*r;
	//	const float h = mCylinderHalfHeight * 2.0f;
	//	const float h2 = h * h;

	//	float cylinder_mass = kVxPi * h * r2 * mDensity;
	//	float hemisphere_mass = (2.0f / 3.0f) * kVxPi * r2 * mRadius * mDensity;


	//	//cylinder
	//	float height_sq = VxSqr(h);
	//	float Iy = r2 * cylinder_mass * 0.5f;
	//	float Ix = cylinder_mass * (3.0f * r2 * height_sq) / 12.0f;

	//	// From hemispheres
	//	const float temp = h * 0.5f + (3.0f/8.0f)*r;

	//	const float hemiI = 0.4f * hemisphere_mass * r2;
	//	Ix += 2.0f * (hemiI + hemisphere_mass * temp * temp);
	//	Iy += 2.0f * hemiI;


	//	const float Mtotal = cylinder_mass + hemisphere_mass * 2.0f;

	//	MassProperties mp;
	//	mp.mass = Mtotal;
	//	mp.inertialTensorDiagonal = Float3(Ix, Iy, Ix);
	//	return mp;
	//}

	Float3 CapsuleShape::ComputeInertiaTensorDiagonal(float mass) const
	{
		const float radius_sq = mRadius * mRadius;
		const float h = mCylinderHalfHeight * 2.0f;
		const float h_sq = h * h;

		//cylinder
		float height_sq = VxSqr(h);
		float Iy = radius_sq * mass * 0.5f;
		float IxIz = mass * 3.0f * radius_sq + h_sq / 12.0f;

		 return Float3(IxIz, Iy, IxIz);
	}
} //namespace vx