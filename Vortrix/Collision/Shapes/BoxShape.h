#pragma once

#include "Shape.h"
#include <array>

namespace vx {



	class BoxShapeSettings final : public ShapeSettings
	{
	public:
		BoxShapeSettings() = default;
		explicit BoxShapeSettings(float h_xyz) : mHalfExtent(h_xyz) { }
		explicit BoxShapeSettings(float h_xyz, float density) : ShapeSettings(density), mHalfExtent(h_xyz) { }

		explicit BoxShapeSettings(const Vec3& half_extents) :  mHalfExtent(half_extents) { }
		explicit BoxShapeSettings(const Vec3& half_extents, float density) :  ShapeSettings(density), mHalfExtent(half_extents) { }
		
		Vec3 mHalfExtent = Vec3(0.5f);
	};



	class BoxShape : public Shape
	{
	public:
		explicit BoxShape(float h_xyz) :
			Shape(EShapeType::Box), mHalfExtent(h_xyz) {
		}
		explicit BoxShape(float hx, float hy, float hz) :
			Shape(EShapeType::Box), mHalfExtent(hx, hy, hz) {
		}

		explicit BoxShape(const Vec3& half_extents) :
			Shape(EShapeType::Box), mHalfExtent(half_extents) {
		}

		explicit BoxShape(const BoxShapeSettings& settings) :
			Shape(EShapeType::Box, settings), mHalfExtent(settings.mHalfExtent) {
		}

		static constexpr const char* GetDebugName() { return "Box"; }

		virtual const char* GetName() const override { return "Box"; }
		virtual Vec3 GetHalfExtents() const override { return Vec3(mHalfExtent); }

		virtual AABB GetLocalBounds() const;
		std::array<Vec3, 8> GetCorners() const;
		std::array<Vec3, 8> GetCornersWS(const Mat44& in_transform) const;

		static void GetSupportingFaceVertices(const Vec3& direction,
			const Vec3& box_min, const Vec3& box_max,
			std::array<Vec3, 4>& out_vertices);

		std::tuple<Vec3, Vec3> GetMinMaxOrientedWS(const Mat44& in_transform) const;

		VX_INLINE std::array<Vec3, 4> GetSupportingFaceVertices(const Vec3& direction) const
		{
			std::array<Vec3, 4> supporting_vertices;
			GetSupportingFaceVertices(direction, -mHalfExtent, mHalfExtent, supporting_vertices);
			return supporting_vertices;
		}

		/// excepts the direction in world space 
		/// to solve 
		/// the direction is tranform into Box local space as AABB solving
		/// retrived supporting face is transformed into world space
		std::array<Vec3, 4> GetSupportingFaceVerticesOBB(const Mat44& in_transform, const Vec3& direction) const;



		Vec3 GetSupportWS(const Mat44& in_transform, const Vec3& dir) const;
		Vec3 GetEdgeCenter(const Mat44& in_transform, int edge_idx, const Vec3& normal) const;
		virtual MassProperties GetMassProperties() const override;
		virtual Float3 ComputeInertiaTensorDiagonal(float mass) const override;

		bool DataEq(const Shape* rhs) const override
		{
			if (rhs == nullptr || GetType() != rhs->GetType()) return false;

			const BoxShape* box_rhs = static_cast<const BoxShape*>(rhs);

			return VxApprox(mDensity, box_rhs->mDensity) && mHalfExtent.IsApprox(box_rhs->mHalfExtent);
		}

	private:
		Vec3 mHalfExtent = Vec3(0.5f);
	};
}