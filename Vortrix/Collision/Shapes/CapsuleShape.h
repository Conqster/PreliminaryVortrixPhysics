#pragma once
#include "Shape.h"

namespace vx {

	struct AABB;

	class CapsuleShape : public Shape
	{
	public:
		explicit CapsuleShape(float radius, float cylinder_half_height) :
			Shape(EShapeType::Capsule), mCylinderHalfHeight(cylinder_half_height), mRadius(radius) {
		}

		virtual Vec3 GetHalfExtents() const override
		{
			float total_height = mCylinderHalfHeight + mRadius;
			return Vec3(mRadius, total_height, mRadius);
		}

		static constexpr const char* GetDebugName() { return "Capsule"; }

		virtual AABB GetLocalBounds() const override;

		virtual AABB GetWorldBounds(const Mat44& tranform, const Vec3& scale) const override;

		Vec3 GetLocalAxis() const { return Vec3(0.0f, 1.0f, 0.0f); }

		Vec3 SupportWS(const Mat44& in_transform, Vec3 dir) const;

		float GetRadius() const { return mRadius; }
		float GetCylinderHalfHeight() const { return mCylinderHalfHeight; }
		float GetTotalHalfHeight() const { return mCylinderHalfHeight + mRadius; }


		virtual MassProperties GetMassProperties() const override;

		bool DataEq(const Shape* rhs) const override
		{
			if (rhs == nullptr || GetType() != rhs->GetType()) return false;

			const CapsuleShape* capsule_rhs = static_cast<const CapsuleShape*>(rhs);

			return mDensity, capsule_rhs->mDensity &&
				mRadius == capsule_rhs->mRadius &&
				mCylinderHalfHeight == capsule_rhs->mCylinderHalfHeight;
		}

	private:
		float mRadius;
		float mCylinderHalfHeight;
	};
#pragma endregion

} //namespace vx