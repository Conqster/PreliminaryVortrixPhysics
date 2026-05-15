#pragma once

#include "Vortrix/Maths/Vec3.h"

namespace vx::Geometry {

	/// Retuen the pairs of the closest point (c0 on seg S0, c1 on seg S1)
	/// p -> start, q -> end
	/// S0(s) = p0 + s*(q0-p0)
	/// S1(t) = p1 + t*(q1-p1)
	/// 
	/// @param p0 & q0 -> segment S0 start & end resp
	///	@param p1 & q1 -> segment S1 start & end resp
	/// 
	/// return distance between P(s) and Q(t)
	VX_INLINE float ClosestPtSegmentSegment(const Vec3& p0, const Vec3& q0,
		const Vec3& p1, const Vec3& q1,
		Vec3& c0, Vec3& c1, float epsilon = 1e-8f)
	{

		Vec3 dS0 = q0 - p0; //dir on S0
		Vec3 dS1 = q1 - p1;
		Vec3 r = p0 - p1;


		float a = dS0.Dot(dS0);//sq S1 len
		float e = dS1.Dot(dS1);
		float f = dS1.Dot(r);

		float s, t;

		if (a <= epsilon && e <= epsilon)
		{
			s = t = 0.0f;
			c0 = p0;
			c1 = p1;
			Vec3 d10 = c0 - c1;
			return d10.LengthSq();
		}
		else if (a <= epsilon)
		{
			s = 0.0f;
			t = VxClamp01(f / e);
		}
		else
		{
			float c = dS0.Dot(r);
			if (e <= epsilon)
			{
				t = 0.0f;
				s = VxClamp01(-c / a);
			}
			else
			{
				float b = dS0.Dot(dS1);
				float denom = a * e - b * b;
				//if (denom < 1e-4f) //fail lets face-face handle in following frame
				//	return false;

				if (denom != 0.0f)
					s = VxClamp01((b * f - c * e) / denom);
				else
					s = 0.0f;

				//if (denom > 1e-6f)
				//	s = VxClamp01((b * f - c * e) / denom);
				//else
				//{
				//	/// range along seg 0
				//	/// mid for stable pt
				//	float s0 = VxClamp01(-c / a);
				//	float s1 = VxClamp01((b - c) / a);
				//	s = (s0 + s1) * 0.5f;
				//}

				float t_nom = (b * s + f);
				if (t_nom < 0.0f)
				{
					t = 0.0f;
					s = VxClamp01(-c / a);
				}
				//else if (t_nom > 1.0f)
				else if (t_nom > e)
				{
					t = 1.0f;
					s = VxClamp01((b - c) / a);
				}
				else
					t = t_nom / e;
			}
		}

		c0 = p0 + dS0 * s;
		c1 = p1 + dS1 * t;
		Vec3 d10 = c0 - c1;
		return d10.LengthSq();
	}

	/// Returns the closest point d and outs the t for the position d
	/// p -> start, q -> end
	/// segment Sab
	/// 
	/// 
	///	@param point -> point
	/// @param a & b -> segment S start & end resp
	/// 
	/// 
	/// @return closest point d on ab -> d(t) = a + t*(b-a)
	VX_INLINE Vec3 ClosestPtPointSegment(const Vec3& point,
		const Vec3& a, const Vec3& b, float& t, float epsilon = 1e-8f)
	{
		Vec3 d;
		Vec3 ab = b - a;
		//Project point ab, but deferring divide by ab.Dot(ab)
		t = (point - a).Dot(ab);
		if (t <= epsilon)
		{
			/// point projects outside the [a,b] interval, on the a side; clamp to a
			t = 0.0f;
			d = a;
		}
		else
		{
			float denom = ab.Dot(ab);//Alway non-negative since denom = ||ab||^2
			if (t >= denom)
			{
				//point projects outside the [a,b] interval, on the b side; clamp to b
				t = 1.0f;
				d = b;
			}
			else
			{
				/// point projects inside the [a,b] interavl; must do deferred divide now
				t = t / denom;
				d = a + t * ab;
			}
		}
		return d;
	}






	VX_INLINE bool RayAABB(const Vec3& origin, const Vec3& inv_dir,
		const Vec3& bounds_min, const Vec3& bounds_max,
		float& _t_min, float t_max_limit)
	{
		float t1 = (bounds_min.X() - origin.X()) * inv_dir.X();
		float t2 = (bounds_max.X() - origin.X()) * inv_dir.X();
		float t_min = VxMin(t1, t2);
		float t_max = VxMax(t1, t2);

		float t3 = (bounds_min.Y() - origin.Y()) * inv_dir.Y();
		float t4 = (bounds_max.Y() - origin.Y()) * inv_dir.Y();
		t_min = VxMax(t_min, VxMin(t3, t4));
		t_max = VxMin(t_max, VxMax(t3, t4));

		float t5 = (bounds_min.Z() - origin.Z()) * inv_dir.Z();
		float t6 = (bounds_max.Z() - origin.Z()) * inv_dir.Z();
		t_min = VxMax(t_min, VxMin(t5, t6));
		t_max = VxMin(t_max, VxMax(t5, t6));
		
		if (t_max <0 || t_min>t_max)
			return false;
		if (t_min > t_max_limit)
			return false;
		_t_min = t_min;
		return true;
	}

	VX_INLINE bool RayAABB(const Vec3& origin, const Vec3& inv_dir,
		const Vec3& bounds_min, const Vec3& bounds_max,
		Float3& axis_min)
	{
		//simd 
		Vec3 t1 = (bounds_min - origin) * inv_dir;
		Vec3 t0 = (bounds_max - origin) * inv_dir;

		Vec3 t_mins = Vec3::Min(t1, t0);
		Vec3 t_maxs = Vec3::Max(t1, t0);

		float t_min = t_mins.MaxComponent();
		float t_max = t_maxs.MinComponent();

		t_mins.Store(axis_min);

		if (t_max <0 || t_min>t_max)
			return false;

		return true;
	}


	/// returns if either intersect x, y
	/// and minimum fraction t along ray
	VX_INLINE Vec4 RayAABB_2(const Vec3& origin, const Vec3& inv_dir,
		const Vec4& boundsX, const Vec4& boundsY, const Vec4& boundsZ)
	{
		//simd 
		//Vec4 boundsX; //first 2 min [A min, Bmin, A max, B max]
		//Vec4 boundsY; //first 2 min [A min, Bmin, A max, B max]
		//Vec4 boundsZ; //first 2 min [A min, Bmin, A max, B max]

		//origins 
		Vec4 origin_x = origin.Splat4X();
		Vec4 origin_y = origin.Splat4Y();
		Vec4 origin_z = origin.Splat4Z();

		//inv direction
		Vec4 inv_dir_x = inv_dir.Splat4X();
		Vec4 inv_dir_y = inv_dir.Splat4Y();
		Vec4 inv_dir_z = inv_dir.Splat4Z();

		Vec4 tx_0 = (boundsX.Swizzle<Axis::X, Axis::Y, Axis::X, Axis::Y>() - origin_x) * inv_dir_x;
		Vec4 tx_1 = (boundsX.Swizzle<Axis::Z, Axis::W, Axis::Z, Axis::W>() - origin_x) * inv_dir_x;

		Vec4 ty_0 = (boundsY.Swizzle<Axis::X, Axis::Y, Axis::X, Axis::Y>() - origin_y) * inv_dir_y;
		Vec4 ty_1 = (boundsY.Swizzle<Axis::Z, Axis::W, Axis::Z, Axis::W>() - origin_y) * inv_dir_y;

		Vec4 tz_0 = (boundsZ.Swizzle<Axis::X, Axis::Y, Axis::X, Axis::Y>() - origin_z) * inv_dir_z;
		Vec4 tz_1 = (boundsZ.Swizzle<Axis::Z, Axis::W, Axis::Z, Axis::W>() - origin_z) * inv_dir_z;


		Vec4 tmin_x = Vec4::Min(tx_0, tx_1);
		Vec4 tmin_y = Vec4::Min(ty_0, ty_1);
		Vec4 tmin_z = Vec4::Min(tz_0, tz_1);

		Vec4 tmax_x = Vec4::Max(tx_0, tx_1);
		Vec4 tmax_y = Vec4::Max(ty_0, ty_1);
		Vec4 tmax_z = Vec4::Max(tz_0, tz_1);


		Vec4 t_entry = Vec4::Max(tmin_x, Vec4::Max(tmin_y, tmin_z));
		Vec4 t_exit = Vec4::Min(tmax_x, Vec4::Min(tmax_y, tmax_z));

		float tEa = t_entry.X();
		float tEb = t_entry.Y();
		float tXa = t_exit.X();
		float tXb = t_exit.Y();

		bool hit_a = (tEa <= tXa) && (tXa >= 0.0f);
		bool hit_b = (tEb <= tXb) && (tXb >= 0.0f);

		float a = hit_a ? tEa : kMaxf;
		float b = hit_b ? tEb : kMaxf;

		return Vec4(a, b, 0.0f, 0.0f);

		//Vec4 tx_min_max = (boundsX - origin_x) * inv_dir_x;
		//Vec4 ty_min_max = (boundsY - origin_y) * inv_dir_y;
		//Vec4 tz_min_max = (boundsZ - origin_z) * inv_dir_z;

		////bad need fixign 
		////x, y, defines the mins for each AABB
		//Vec4 tx_mins = Vec4::Min(tx_min_max, tx_min_max.Swizzle<Axis::Z, Axis::W, Axis::W, Axis::W>());
		//Vec4 ty_mins = Vec4::Min(ty_min_max, ty_min_max.Swizzle<Axis::Z, Axis::W, Axis::W, Axis::W>());
		//Vec4 tz_mins = Vec4::Min(tz_min_max, tz_min_max.Swizzle<Axis::Z, Axis::W, Axis::W, Axis::W>());
		//

		////x, y, defines the maxs for each AABB
		//Vec4 tx_maxs = Vec4::Max(tx_min_max, tx_min_max.Swizzle<Axis::Z, Axis::W, Axis::W, Axis::W>());
		//Vec4 ty_maxs = Vec4::Max(ty_min_max, ty_min_max.Swizzle<Axis::Z, Axis::W, Axis::W, Axis::W>());
		//Vec4 tz_maxs = Vec4::Max(tz_min_max, tz_min_max.Swizzle<Axis::Z, Axis::W, Axis::W, Axis::W>());

		////aabb1
		//Vec3 t_mins1 = Vec3(tx_mins.X(), ty_mins.X(), tz_mins.X());
		//Vec3 t_maxs1 = Vec3(tx_maxs.X(), ty_maxs.X(), tz_maxs.X());
		////aabb2
		//Vec3 t_mins2 = Vec3(tx_mins.Y(), ty_mins.Y(), tz_mins.Y());
		//Vec3 t_maxs2 = Vec3(tx_maxs.Y(), ty_maxs.Y(), tz_maxs.Y());

		//float t_min1 = t_mins1.MaxComponent();
		//float t_max1 = t_maxs1.MinComponent();
		//bool intersect1 = (t_max1 <0 || t_min1>t_max1);


		//float t_min2 = t_mins2.MaxComponent();
		//float t_max2 = t_maxs2.MinComponent();
		//bool intersect2 = (t_max2 <0 || t_min2>t_max2);

		//return Vec4(intersect1, intersect2, t_min1, t_min2);


		//Vec3 t1 = (bounds_min - origin) * inv_dir;
		//Vec3 t0 = (bounds_max - origin) * inv_dir;

		//Vec3 t_mins = Vec3::Min(t1, t0);
		//Vec3 t_maxs = Vec3::Max(t1, t0);

		//float t_min = t_mins.MaxComponent();
		//float t_max = t_maxs.MinComponent();

		//t_mins.Store(axis_min);

		//if (t_max <0 || t_min>t_max)
		//	return false;

		//return true;
	}



	VX_INLINE bool RayAABB(const Vec3& origin, const Vec3& inv_dir,
		const Vec3& bounds_min, const Vec3& bounds_max,
		float& _t_min, Vec3& o_point)
	{
	
		_t_min = 0.0f;
		float t_max = kMaxf;
		for (int i = 0; i < 3; i++)
		{
			if (VxAbs(inv_dir[i] < kEpsilon))
			{
				if (origin[i]<bounds_min[i] || origin[i] > bounds_max[i]) return false;
			}
			else
			{
				float ood = 1.0f / inv_dir[i];
				float t1 = (bounds_min[i] - origin[i]) * ood;
				float t2 = (bounds_max[i] - origin[i]) * ood;

				if (t1 > t2)
					std::swap(t1, t2);

				_t_min = VxMax(_t_min, t1);
				t_max = VxMin(t_max, t2);
				if (_t_min > t_max) return false;
			}
		}
		o_point = origin + inv_dir * _t_min;
		return true;
	}
}