#pragma once
#include "Shape.h"

namespace vx {

	struct AABB;

	class CapsuleShapeSettings final : public ShapeSettings
	{
	public:

		explicit CapsuleShapeSettings(float radius, float cylinder_half_height) : mCylinderHalfHeight(cylinder_half_height), mRadius(radius) {}

	public:
		float mCylinderHalfHeight = 0.5f;
		float mRadius = 0.5f;
	};


	class CapsuleShape : public Shape
	{
	public:
		explicit CapsuleShape(float radius, float cylinder_half_height) :
			Shape(EShapeType::Capsule), mCylinderHalfHeight(cylinder_half_height), mRadius(radius) {
		}

		explicit CapsuleShape(const CapsuleShapeSettings& settings) :
			Shape(EShapeType::Capsule, settings), mCylinderHalfHeight(settings.mCylinderHalfHeight), mRadius(settings.mRadius) {
		}

		virtual Vec3 HalfExtents() const override
		{
			float total_height = mCylinderHalfHeight + mRadius;
			return Vec3(mRadius, total_height, mRadius);
		}

		static constexpr const char* GetDebugName() { return "Capsule"; }

		virtual AABB LocalBounds() const override;

		virtual AABB ComputeWorldBounds(const Mat44& tranform, const Vec3& scale) const override;

		Vec3 GetLocalAxis() const { return Vec3(0.0f, 1.0f, 0.0f); }

		Vec3 SupportWS(const Mat44& in_transform, Vec3 dir) const;

		float GetRadius() const { return mRadius; }
		float GetCylinderHalfHeight() const { return mCylinderHalfHeight; }
		float GetTotalHalfHeight() const { return mCylinderHalfHeight + mRadius; }


		virtual MassProperties GetMassProperties() const override;
		virtual Float3 ComputeInertiaTensorDiagonal(float mass) const override;

		bool DataEq(const Shape* rhs) const override
		{
			if (rhs == nullptr || Type() != rhs->Type()) return false;

			const CapsuleShape* capsule_rhs = static_cast<const CapsuleShape*>(rhs);

			return mDensity == capsule_rhs->mDensity &&
				mRadius == capsule_rhs->mRadius &&
				mCylinderHalfHeight == capsule_rhs->mCylinderHalfHeight;
		}

	private:
		float mRadius;
		float mCylinderHalfHeight;
	};
#pragma endregion

} //namespace vx