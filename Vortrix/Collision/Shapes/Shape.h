#pragma once

#include "Vortrix.h"
#include "Geometry/AABB.h"

namespace vx
{
	
	enum class EShapeType : uint8
	{
		Sphere,
		Box,
		Capsule,
		Plane,

		Count
	};

	static constexpr const char* GetEShapeTypeName(const EShapeType type)
	{
		switch (type)
		{
		case EShapeType::Box: return "Box";
		case EShapeType::Sphere: return "Sphere";
		case EShapeType::Plane: return "Plane";
		case EShapeType::Capsule: return "Capsule";
		default: return "Unk";
		}
	}

	/// used as a hank shake between shape and body
	struct MassProperties
	{
		/// diagonal moment inertia at the moment only supports 
		/// basic convex shapes, with uniform distributed mass
		Float3 inertialTensorDiagonal{0.0f};
		float mass = 0.0f;
	};


	class Shape
	{
	public:
		EShapeType GetType() const { return mType; }
		constexpr const char* GetShapeTypeName() const { return GetEShapeTypeName(mType); }
		virtual const char* GetName() const { return "Base"; }

		virtual AABB GetLocalBounds() const = 0;
		virtual AABB GetWorldBounds(const Mat44& tranform, const Vec3& scale) const { return GetLocalBounds().Scaled(scale).Transformed(tranform); }

		void SetDensity(float density) { mDensity = density; }
		float GetDensity() { return mDensity; }

		virtual MassProperties GetMassProperties() const = 0; 

		///GetHalfScale 
		/// for rendering
		virtual Vec3 GetHalfExtents() const = 0;


		virtual bool DataEq(const Shape* rhs) const = 0;

		virtual ~Shape() {}
	protected:
		Shape(EShapeType type) : mType(type) {}
		EShapeType mType;

		/// kg/m^3
		float mDensity = 1000.0f;
	};
#pragma endregion
	
}