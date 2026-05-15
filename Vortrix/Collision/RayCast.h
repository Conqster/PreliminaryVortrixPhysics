#pragma once


#include "Vortrix/Maths/Vec3.h"

namespace vx {
		

	struct RayCast
	{
		Vec3 origin;
		Vec3 displacement;// to work with hitfraction not a unit direction

		Vec3 direction;
		Vec3 invDisplacement;

		float length;

		RayCast() = default;

		RayCast(const Vec3& _origin, const Vec3& _displacment) :
			origin(_origin), displacement(_displacment)
		{
			length = displacement.Length();
			direction = displacement / length;
			invDisplacement[0] = (std::abs(displacement.X()) > 1e-9f) ? (1.0f / displacement.X()) : 1e30f;
			invDisplacement[1] = (std::abs(displacement.Y()) > 1e-9f) ? (1.0f / displacement.Y()) : 1e30f;
			invDisplacement[2] = (std::abs(displacement.Z()) > 1e-9f) ? (1.0f / displacement.Z()) : 1e30f;
		}


		VX_INLINE Vec3 End() const { return origin + displacement; }
		//Vec3 End() const { return origin + direction * length; }



		VX_INLINE Vec3 PointAlongRay(float fraction) const
		{
			return origin + displacement * fraction;
		}
	};
}

