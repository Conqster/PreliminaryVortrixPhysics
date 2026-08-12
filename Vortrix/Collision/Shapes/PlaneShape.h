#pragma once

#include "Shape.h"

namespace vx {

	struct AABB;

	class PlaneShapeSettings final : public ShapeSettings
	{
	public:
		explicit PlaneShapeSettings(const Vec3& nor, float half_extent) : mNormal(nor), mHalfExtent(half_extent){}
	public:
		Vec3 mNormal;
		float mHalfExtent = 0.0f;
		float mConstant = 0.0f;
	};

	class PlaneShape : public Shape
	{
	public:
		explicit PlaneShape(const Vec3& nor, float half_extent) :
			Shape(EShapeType::Plane), mNormal(nor), mHalfExtent(half_extent)
		{
			ComputeLocalBounds();
		}

		explicit PlaneShape(const PlaneShapeSettings& settings) :
			Shape(EShapeType::Plane, settings), mNormal(settings.mNormal), mHalfExtent(settings.mHalfExtent), mConstant(settings.mConstant)
		{
			ComputeLocalBounds();
		}

		static constexpr const char* GetDebugName() { return "Plane"; }
		virtual Vec3 HalfExtents() const override { return Vec3(mHalfExtent); }
		const Vec3 Normal() const { return mNormal; }
		const float GetOffset() const { return mConstant; }
		virtual AABB LocalBounds() const override { return mLocalBounds; }

		virtual MassProperties GetMassProperties() const override { return {}; }
		virtual Float3 ComputeInertiaTensorDiagonal(float mass) const override { return {}; }

		AABB ComputeLocalBounds();

		bool DataEq(const Shape* rhs) const override
		{
			if (rhs == nullptr || Type() != rhs->Type()) return false;

			const PlaneShape* plane_rhs = static_cast<const PlaneShape*>(rhs);

			return mDensity && plane_rhs->mDensity&&
				mHalfExtent == plane_rhs->mHalfExtent &&
				mConstant == plane_rhs->mConstant &&
				mNormal.IsApprox(plane_rhs->mNormal);
		}

	private:
		Vec3 mNormal;
		float mHalfExtent = 0.0f;
		float mConstant = 0.0f;

		AABB mLocalBounds;
	};
} //namespace vx