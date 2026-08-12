#pragma once

#include "Shape.h"

namespace vx {

	class SphereShapeSettings final : public ShapeSettings
	{
	public:
		explicit SphereShapeSettings(float radius) : mRadius(radius){ }
	public:
		float mRadius = 0.5f;
	};

	class SphereShape : public Shape
	{
	public:
		explicit SphereShape(float radius) :
			Shape(EShapeType::Sphere), mRadius(radius) {
		}

		explicit SphereShape(const SphereShapeSettings& settings) :
			Shape(EShapeType::Sphere, settings), mRadius(settings.mRadius) {
		}

		static constexpr const char* GetDebugName() { return "Sphere"; }
		VX_INLINE float GetRadius() const { return mRadius; }
		VX_INLINE virtual Vec3 HalfExtents() const override { return Vec3(mRadius); }

		virtual AABB LocalBounds() const override;

		virtual MassProperties GetMassProperties() const override;
		virtual Float3 ComputeInertiaTensorDiagonal(float mass) const override;

		virtual AABB ComputeWorldBounds(const Mat44& tranform, const Vec3& scale) const override;

		bool DataEq(const Shape* rhs) const override
		{
			if (rhs == nullptr || Type() != rhs->Type()) return false;

			const SphereShape* sphere_rhs = static_cast<const SphereShape*>(rhs);

			return mDensity, sphere_rhs->mDensity &&
				mRadius == sphere_rhs->mRadius;
		}

	private:
		float mRadius;
	};

}//namespace vx