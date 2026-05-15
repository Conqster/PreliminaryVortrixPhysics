#pragma once
#pragma once


#include "Maths/Vec3.h"
#include "Maths/Mat44.h"

#include <array>

namespace vx
{
	struct Sphere
	{
		Vec3 center = Vec3(0.0f);
		real radius = 0.0f;

		//Constructor
		Sphere() = default;
		Sphere(const Vec3& c, float r) :center(c), radius(r) {}
		explicit Sphere(float r) : center(Vec3(0.0f)), radius(r) {}


		float GetVolume() const 
		{
			constexpr float f = (4.0f / 3.0f) * kPI;
			return f * radius * radius * radius;
		}
		float GetSurfaceArea() const
		{
			constexpr float f = 4.0f * kPI;
			return f * radius * radius;
		}
		bool IsValid() const 
		{
			//fix vec3 class does not have is finite yet
			auto constexpr is_finite = [](const Vec3& v) {
				return std::isfinite(v.X()) &&
					std::isfinite(v.Y()) && std::isfinite(v.Z());
			};
			return radius >= 0.0f && is_finite(center);
		}
		bool Contains(const Vec3& pointA) const { return (pointA - center).LengthSq() <= radius * radius; }

		bool Overlaps(const Sphere& rhs) const
		{
			const float dist_sq = (center - rhs.center).LengthSq();
			const float radius_sum = radius + rhs.radius;
			return dist_sq <= radius_sum * radius_sum;
		}
		void Merge(const Sphere& rhs)
		{
			Vec3 delta = rhs.center - center;
			float dist = delta.Length();
			
			if (dist + rhs.radius <= radius) return; //alerady contain other
			if (dist + radius <= rhs.radius) 
			{
				center = rhs.center;
				radius = rhs.radius;
				return;
			}

			Vec3 dir = delta / dist;
			Vec3 a = center - dir * radius;
			Vec3 b = center + dir * rhs.radius;
			center = (a + b) * 0.5f;
			radius = (b - a).Length() * 0.5f;
		}
		Sphere Merged(const Sphere& rhs) const
		{
			Sphere result = *this;
			result.Merge(rhs);
			return result;
		}
		void Translate(const Vec3& translation) { center += translation; }
		//need to scale as well later matching AABB
		void Transform(const Mat44& transform) 
		{ 
			center = transform.Transform(center); 
			//Vec3 scale = transform.Get
			//	float max_scale = Max(scale.X(),scale.Y(),scale.Z())
		}


		//utilities tool, 
		//if used as part of template to match AABB calls
		//i might have an interface which enforce implementation fo bound shape
		Vec3 GetCenter() const { return center; }
		Vec3 GetSize() const { return Vec3(radius * 2.0f); }
		Vec3 GetExtent() const { return Vec3(radius); }
		void Expand(const Vec3& scale) { radius += scale.Length(); }
		void Grow(const float size) { radius += size; }
		void Merge(const Vec3& pointA)
		{
			const float dist = (center - pointA).Length();
			radius = Max(radius, dist);
		}
		void Reset() { radius = -1.0f; }

		AABB ToABBB() const
		{
			return AABB(center - Vec3(radius), center + Vec3(radius));
		}
	};
}