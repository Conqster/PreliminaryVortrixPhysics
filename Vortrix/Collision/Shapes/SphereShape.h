#pragma once

#include "Shape.h"

namespace vx {
	class SphereShape : public Shape
	{
	public:
		explicit SphereShape(float radius) :
			Shape(EShapeType::Sphere), mRadius(radius) {
		}

		static constexpr const char* GetDebugName() { return "Sphere"; }
		VX_INLINE float GetRadius() const { return mRadius; }
		VX_INLINE virtual Vec3 GetHalfExtents() const override { return Vec3(mRadius); }

		virtual AABB GetLocalBounds() const override;

		virtual MassProperties GetMassProperties() const override;

		virtual AABB GetWorldBounds(const Mat44& tranform, const Vec3& scale) const override;

		bool DataEq(const Shape* rhs) const override
		{
			if (rhs == nullptr || GetType() != rhs->GetType()) return false;

			const SphereShape* sphere_rhs = static_cast<const SphereShape*>(rhs);

			return mDensity, sphere_rhs->mDensity &&
				mRadius == sphere_rhs->mRadius;
		}

	private:
		float mRadius;
	};

}//namespace vx