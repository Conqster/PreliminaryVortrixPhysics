#include "SphereShape.h"
#include "Vortrix/Geometry/AABB.h"

namespace vx {
	AABB SphereShape::LocalBounds() const
	{
		Vec3 half_extent(mRadius);
		return AABB(-half_extent, half_extent);
	}
	MassProperties SphereShape::GetMassProperties() const
	{
		if (mDensity == 0.0f) return {};
		float radius_sq = mRadius * mRadius;

		/// density kg/m^3
		/// density (rho p)
		/// p = mass/vol 
		/// mass = p x V
		/// 
		float mass = ((4.0f / 3.0f) * kVxPi * mRadius * radius_sq) * mDensity;
		///sphere inretia tensor (isotropic) same value no matter location
		/// inetia scalar = 2/5 * mass * radius * radius
		float inertia_scalar = (2.0f / 5.0f) * mass * radius_sq;

		MassProperties mp;
		mp.mass = mass;
		mp.inertialTensorDiagonal = Float3(inertia_scalar);
		return mp;
	}
	Float3 SphereShape::ComputeInertiaTensorDiagonal(float mass) const
	{
		float radius_sq = mRadius * mRadius;
		return Float3((2.0f / 5.0f) * mass * radius_sq);
	}
	AABB SphereShape::ComputeWorldBounds(const Mat44& tranform, const Vec3& scale) const
	{
		Vec3 scaled_radius = scale.Abs() * mRadius;
		scaled_radius = scaled_radius.SplatX();
		AABB bounds(-scaled_radius, scaled_radius);
		bounds.Translate(tranform.GetTranslation()); //ignore rot
		return bounds;
	}
} //namespace vx