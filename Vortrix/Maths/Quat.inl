#include "Quat.h"
#include "Mat44.h"
#include "SimdUtil.h"

#include "Vortrix/Core/Logger.h"

namespace vx
{
	inline VX_INLINE Quat Quat::FromAxisAngle(const Vec3& axis, float angle)
	{
		/// imaginary xyz: axis * sin(half_angle) 
		/// real w:				  cos(half_angle)
 		VX_ASSERT(axis.IsNormalised(), "Axis must be non-zero & normalised");

		float h = 0.5f * angle;//half
		float s = VxSin(h);

		return Quat(axis * s, VxCos(h));
	}
	inline VX_INLINE void Quat::SetAxisAngle(const Vec3& axis, float angle)
	{
		/// imaginary xyz: axis * sin(half_angle) 
		/// real w:				  cos(half_angle)
		VX_ASSERT(axis.IsNormalised(), "Axis must be non-zero & normalised");

		float h = 0.5f * angle;//half
		float s = VxSin(h);

		mValue = Vec4(axis * s, VxCos(h));
	}
	inline VX_INLINE void Quat::GetAxisAngle(Vec3& o_axis, float& o_angle)
	{
		VX_ASSERT(IsUnitQuat(), "Quat must be normalised");

		//shortest angle
		float w = VxMin(VxAbs(W()), 1.0f);
		
		o_angle = 2.0f * VxAcos(w);

		float s_sq = 1.0f - w * w;//==sin^2(angle/2)

		if (s_sq < kEpsilon * kEpsilon)
			o_axis = Vec3::Zero();
		else
			o_axis = (1.0f / VxSqrt(s_sq)) * Imaginary();
	}
	VX_INLINE void Quat::Normalise()
	{
		float len = mValue.Length();

		if (len < kEpsilon)
		{
			mValue.SetW(1.0f);
			return;
		}

#ifdef VX_SIMD_SSE
		mValue /= len; //<- sse path div
#else
		len = 1.0f / len; //<- scalar inv 
		mValue *= len;	//<- scalar mul
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Quat Quat::Normalised() const
	{
		Quat q(*this);
		q.Normalise();
		return q;
	}

	inline VX_INLINE bool Quat::IsUnitQuat(float tolerance) const
	{
		return VxAbs(LengthSq() - 1.0f) <= tolerance;
	}

	inline VX_INLINE Quat Quat::Inversed() const
	{
		return Conjugated() / Length();
	}

	inline VX_INLINE Vec3 Quat::Rotate(const Vec3& vec) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		/// rotate fast
		/// v' = q * q(v) * q^-1
		///			  = q * q(v.xyz, 0.0f) * q^-1
		///
		/// rodrigues:
		///  x: cross, *: dot
		/// v'= v + 2 * w * (q.xyz x v) + 2 * (q.xyz x (q.xyz x v))
		/// simplify 
		/// 
		/// let t = (q.xyz x v)
		/// v'= v + 2 * w * t + 2 * (q.xyz x t)
		/// 
		/// 
		/// t = 2(q.xyz x v)
		/// t = (q.xyz x v) + (q.xyz x v)
		/// i.e t + t  = (q.xyz x v)
		/// 
		/// v' = v +  w * t + (q.xyz x t)
		/// v' = v + wt + (q.xyz x t)
		/// 
		/// 

		Vec3 q_xyz = Imaginary();
		Vec3 t = q_xyz.Cross(vec);
		t += t;
		return vec + W() * t + q_xyz.Cross(t);
	}

	inline VX_INLINE Vec3 Quat::InverseRotate(const Vec3& vec) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		Vec3 q_xyz = Imaginary();
		q_xyz.FlipSignAssign<-1, -1, -1>();
		Vec3 t = q_xyz.Cross(vec);
		t += t;
		return vec + W() * t + q_xyz.Cross(t);
	}

	inline VX_INLINE Vec3 Quat::RotateSlow(const Vec3& vec) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		/// v' = q * q(v) * q^-1
		Quat qv(vec, 0.0f);
		Quat v = (*this) * qv * Conjugated();
		return v.Imaginary();
	}

	inline VX_INLINE Vec3 Quat::InverseRotateSlow(const Vec3& vec) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		/// v' = q^-1 * q(v) * q
		Quat qv(vec, 0.0f);
		Quat v = Conjugated() * qv * (*this);
		return v.Imaginary();
	}

	inline VX_INLINE Vec3 Quat::RotateAxisX() const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		return RotateScaledAxisX(1.0f);

		float x = X(), y = Y(), z = Z(), w = W();

		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		//float xx = x * tx;
		float yy = ty * y;
		float zz = tz * z;

		float xy = tx * y; //ty * x;
		float zw = tz * w;

		float xz = tx * z;
		float yw = ty * w;


		return Vec3(1.0f - (yy + zz), xy + zw, xz - yw);
	}

	inline VX_INLINE Vec3 Quat::RotateAxisY() const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		float x = X(), y = Y(), z = Z(), w = W();

		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		float xy = tx * y; //ty * x;
		float zw = tz * w;

		float zz = tz * z;
		float xx = tx * x;

		float yz = ty * z;
		float xw = tx * w;

		return Vec3(xy - zw, (1.0f - zz) - xx, yz + xw);
	}

	inline VX_INLINE Vec3 Quat::RotateAxisZ() const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		float tx = X() + X();
		float ty = Y() + Y();
		float tz = Z() + Z();

		float xz = tx * Z(); //ty * x;
		float yw = ty * W();

		float yz = ty * Z();
		float xw = tx * W();

		float xx = tx * X();
		float yy = ty * Y();

		return Vec3(xz + yw, yz - xw, (1.0f - xx) - yy);
	}

	inline VX_INLINE Vec3 Quat::RotateScaledAxisX(float scale) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		float x = X(), y = Y(), z = Z(), w = W();

		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		//float xx = x * tx;
		float yy = ty * y;
		float zz = tz * z;

		float xy = tx * y; //ty * x;
		float zw = tz * w;

		float xz = tx * z;
		float yw = ty * w;

		return Vec3(scale * 1.0f - (yy + zz),
					scale * xy + zw, 
					scale * xz - yw);
	}

	inline VX_INLINE Vec3 Quat::RotateScaledAxisY(float scale) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		float x = X(), y = Y(), z = Z(), w = W();

		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		float xy = tx * y; //ty * x;
		float zw = tz * w;

		float zz = tz * z;
		float xx = tx * x;

		float yz = ty * z;
		float xw = tx * w;

		return Vec3(scale * (xy - zw), scale * ((1.0f - zz) - xx), scale * (yz + xw));
	}

	inline VX_INLINE Vec3 Quat::RotateScaledAxisZ(float scale) const
	{
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");
		float tx = X() + X();
		float ty = Y() + Y();
		float tz = Z() + Z();

		float xz = tx * Z(); //ty * x;
		float yw = ty * W();

		float yz = ty * Z();
		float xw = tx * W();

		float xx = tx * X();
		float yy = ty * Y();

		return Vec3(scale * (xz + yw), scale * (yz - xw), scale * ((1.0f - xx) - yy));
	}

	inline VX_INLINE void Quat::RotateScaledAxes(const Vec3& scale, Vec3& out_x, Vec3& out_y, Vec3& out_z)
	{

		////| [1 - 2(yy - zz) xy + wz        xz - wy] | T
		////| [xy - wz        1-2(xx - zz)]  yz + wx] |
		////| [xz + wy        yz - wx        1 - xx - yy] |

		/// assume Quat is unit quaternion 
		/// 
		////| [1 - (yy - zz)	  xy + wz         xz - wy]	 | T
		////| [	  xy - wz      1 - (xx - zz)]     yz + wx]   |
		////| [   xz + wy		  yz - wx      1 - (xx - yy)]|
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");

#ifdef VX_SIMD_SSE

		__m128 q = mValue.Value();

		//[tx, ty, tz, tw]
		__m128 t_xyz = _mm_add_ps(q, q);
		//[xx, yy, zz, ww]
		__m128 xx_yy_zz_ww = _mm_mul_ps(t_xyz, q);


		/// diag [yy + zz, xx + zz, xx + yy, 0]
		//__m128 diag = _mm_add_ps(simd::Swizzle<1, 0, 0, 3>(xx_yy_zz_ww), simd::Swizzle<2, 2, 1, 3>(xx_yy_zz_ww));
		__m128 diag = _mm_add_ps(simd::Swizzle<simd::Y, simd::X, simd::X, simd::W>(xx_yy_zz_ww), simd::Swizzle<kAxisZ, kAxisZ, kAxisY, kAxisW>(xx_yy_zz_ww));
		/// [1-(yy + zz), 1-(xx + zz), 1-(xx + yy)]
		diag = _mm_sub_ps(_mm_set1_ps(1.0f), diag);

		/// 01: xy + wz
		/// 02: xz - wy
		/// 
		/// 10: xy - wz
		/// 12: yz + wx
		/// 
		/// 20: zz + wy
		/// 21: yz - wx
		/// 
		/// group 
		/// 01: xy + wz
		/// 10: xy - wz
		/// 
		/// 20: xz + wy
		/// 02: xz - wy
		/// 
		/// 12: yz + wx
		/// 21: yz - wx
		/// 
		/// 
		/// a: xy xz yz
		/// b: wz wy wx
		/// 
		/// 01_20_12: a + b
		/// 10_02_21: a - b
	
		//__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<0, 0, 1, 1>(t_xyz), simd::Swizzle<1, 2, 2, 2>(q));
		__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<kAxisX, kAxisX, kAxisY, kAxisY>(t_xyz), simd::Swizzle<kAxisY, kAxisZ, kAxisZ, kAxisZ>(q));
		//__m128 w_zyx = _mm_mul_ps(simd::Swizzle<3, 3, 3, 3>(t_xyz), simd::Swizzle<2, 1, 0, 3>(q));
		//__m128 w_zyx = _mm_mul_ps(simd::Swizzle<3, 3, 3, 3>(q), simd::Swizzle<2, 1, 0, 3>(t_xyz));
		__m128 w_zyx = _mm_mul_ps(simd::Swizzle<kAxisW, kAxisW, kAxisW, kAxisW>(q), simd::Swizzle<kAxisZ, kAxisY, kAxisX, kAxisW>(t_xyz));

		/// require write 
		/// d 01 02
		/// 10 d 12 
		/// 20 21 d
		__m128 r20_01_12 = _mm_add_ps(xy_xz_yz, w_zyx); //01_20_12
		//r20_01_12 = simd::Swizzle<1, 0, 2, 2>(r20_01_12);
		r20_01_12 = simd::Swizzle<kAxisY, kAxisX, kAxisZ, kAxisZ>(r20_01_12);
		__m128 r10_21_02 = _mm_sub_ps(xy_xz_yz, w_zyx); //10_02_21
		//r10_21_02 = simd::Swizzle<0, 2, 1, 1>(r10_21_02);
		r10_21_02 = simd::Swizzle<kAxisX, kAxisZ, kAxisY, kAxisY>(r10_21_02);

		__m128 s = scale.Value();
		__m128 r = _mm_blend_ps(_mm_blend_ps(diag, r20_01_12, 0b1110), r10_21_02, 0b1100);
		//out_x = Vec3(_mm_mul_ps(r, simd::Swizzle<0, 0, 0, 0>(s)));
		out_x = Vec3(_mm_mul_ps(r, simd::Swizzle<kAxisX, kAxisX, kAxisX, kAxisX>(s)));
		r = _mm_blend_ps(_mm_blend_ps(r10_21_02, diag, 0b1110), r20_01_12, 0b1100);
		//out_y = Vec3(_mm_mul_ps(r, simd::Swizzle<1, 1, 1, 1>(s)));
		out_y = Vec3(_mm_mul_ps(r, simd::Swizzle<kAxisY, kAxisY, kAxisY, kAxisY>(s)));
		r = _mm_blend_ps(_mm_blend_ps(r20_01_12, r10_21_02, 0b1110), diag, 0b1100);
		//out_z = Vec3(_mm_mul_ps(r, simd::Swizzle<2, 2, 2, 2>(s)));
		out_z = Vec3(_mm_mul_ps(r, simd::Swizzle<kAxisZ, kAxisZ, kAxisZ, kAxisZ>(s)));

#else
		float tx = X() + X();
		float ty = Y() + Y();
		float tz = Z() + Z();

		float xx = tx * X();
		float yy = ty * Y();
		float zz = tz * Z();

		float xy = tx * Y();
		float xz = tx * Z();
		float xw = tx * W();

		float yz = ty * Z();
		float yw = ty * W();
		float zw = tz * W();

		/// basis axes
		out_x = Vec3(scale.X() * (1.0f - (yy + zz)), scale.X() * (xy + zw), scale.X() * (xz - yw));
		out_y = Vec3(scale.Y() * (xy - zw), scale.Y() * ((1.0f - zz) - xx), scale.Y() * (yz + xw));
		out_z = Vec3(scale.Z() * (xz + yw), scale.Z() * (yz - xw), scale.Z() * ((1.0f - xx) - yy));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Mat44 Quat::GetRotationMat44()
	{

		////| [1 - 2(yy - zz) xy + wz        xz - wy] | T
		////| [xy - wz        1-2(xx - zz)]  yz + wx] |
		////| [xz + wy        yz - wx        1 - xx - yy] |

		/// assume Quat is unit quaternion 
		/// 
		////| [1 - (yy - zz)	  xy + wz         xz - wy]	 | T
		////| [	  xy - wz      1 - (xx - zz)]     yz + wx]   |
		////| [   xz + wy		  yz - wx      1 - (xx - yy)]|
		VX_ASSERT(IsUnitQuat(), "Quaternion is not unit");

#ifdef VX_SIMD_SSE


		__m128 q = mValue.Value();

		//[tx, ty, tz, tw]
		__m128 t_xyz = _mm_add_ps(q, q);
		//[xx, yy, zz, ww]
		__m128 xx_yy_zz_ww = _mm_mul_ps(t_xyz, q);


		/// diag [yy + zz, xx + zz, xx + yy, 0]
		//__m128 diag = _mm_add_ps(simd::Swizzle<1, 0, 0, 3>(xx_yy_zz_ww), simd::Swizzle<2, 2, 1, 3>(xx_yy_zz_ww));
		__m128 diag = _mm_add_ps(simd::Swizzle<kAxisY, kAxisX, kAxisX, kAxisW>(xx_yy_zz_ww), simd::Swizzle<kAxisZ, kAxisZ, kAxisY, kAxisW>(xx_yy_zz_ww));
		/// [1-(yy + zz), 1-(xx + zz), 1-(xx + yy)]
		diag = _mm_sub_ps(_mm_set1_ps(1.0f), diag);

		/// 01: xy + wz
		/// 02: xz - wy
		/// 
		/// 10: xy - wz
		/// 12: yz + wx
		/// 
		/// 20: zz + wy
		/// 21: yz - wx
		/// 
		/// group 
		/// 01: xy + wz
		/// 10: xy - wz
		/// 
		/// 20: xz + wy
		/// 02: xz - wy
		/// 
		/// 12: yz + wx
		/// 21: yz - wx
		/// 
		/// 
		/// a: xy xz yz
		/// b: wz wy wx
		/// 
		/// 01_20_12: a + b
		/// 10_02_21: a - b

		//__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<0, 0, 1, 1>(t_xyz), simd::Swizzle<1, 2, 2, 2>(q));
		__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<kAxisX, kAxisX, kAxisY, kAxisY>(t_xyz), simd::Swizzle<kAxisY, kAxisZ, kAxisZ, kAxisZ>(q));
		//__m128 w_zyx = _mm_mul_ps(simd::Swizzle<3, 3, 3, 3>(q), simd::Swizzle<2, 1, 0, 3>(t_xyz));
		__m128 w_zyx = _mm_mul_ps(simd::Swizzle<kAxisW, kAxisW, kAxisW, kAxisW>(q), simd::Swizzle<kAxisZ, kAxisY, kAxisX, kAxisW>(t_xyz));

		/// require write 
		/// d 01 02
		/// 10 d 12 
		/// 20 21 d
		__m128 r20_01_12 = _mm_add_ps(xy_xz_yz, w_zyx); //01_20_12
		//r20_01_12 = simd::Swizzle<1, 0, 2, 2>(r20_01_12);
		r20_01_12 = simd::Swizzle<kAxisY, kAxisX, kAxisZ, kAxisZ>(r20_01_12);
		__m128 r10_21_02 = _mm_sub_ps(xy_xz_yz, w_zyx); //10_02_21
		//r10_21_02 = simd::Swizzle<0, 2, 1, 1>(r10_21_02);
		r10_21_02 = simd::Swizzle<kAxisX, kAxisZ, kAxisY, kAxisY>(r10_21_02);

		__m128 xC = _mm_blend_ps(_mm_blend_ps(diag, r20_01_12, 0b1110), r10_21_02, 0b1100);
		__m128 yC = _mm_blend_ps(_mm_blend_ps(r10_21_02, diag, 0b1110), r20_01_12, 0b1100);
		__m128 zC = _mm_blend_ps(_mm_blend_ps(r20_01_12, r10_21_02, 0b1110), diag, 0b1100);
		//ensure affine bottom & translate 0, 0,0 1
		__m128 mask = simd::LaneMask<1, 1, 1, 0>();
	
		return Mat44(Vec4(_mm_and_ps(xC, mask)),
					 Vec4(_mm_and_ps(yC, mask)),
					 Vec4(_mm_and_ps(zC, mask)));

#else

		float tx = X() + X();
		float ty = Y() + Y();
		float tz = Z() + Z();

		float xx = tx * X();
		float yy = ty * Y();
		float zz = tz * Z();

		float xy = tx * Y();
		float xz = tx * Z();
		float xw = tx * W();

		float yz = ty * Z();
		float yw = ty * W();
		float zw = tz * W();

		/// basis axes
		return Mat44(Vec4(1.0f - (yy + zz), xy + zw, xz - yw, 0.0f),
			Vec4(xy - zw, (1.0f - zz) - xx, yz + xw, 0.0f),
			Vec4(xz + yw, yz - xw, (1.0f - xx) - yy, 0.0f));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Quat Quat::operator*(const Quat& rhs) const
	{
		Quat r(*this);
		r *= rhs;
		return r;
	}

	inline VX_INLINE Quat Quat::operator*=(const Quat& rhs)
	{
#ifdef VX_SIMD_SSE
		__m128 q1 = mValue.Value();
		__m128 q2 = rhs.mValue.Value();

		//w
		//__m128 w1 = simd::Swizzle<3, 3, 3, 3>(q1);
		//__m128 w2 = simd::Swizzle<3, 3, 3, 3>(q2);
		__m128 w1 = simd::Swizzle<kAxisW, kAxisW, kAxisW, kAxisW>(q1);
		__m128 w2 = simd::Swizzle<kAxisW, kAxisW, kAxisW, kAxisW>(q2);

		__m128 v = _mm_add_ps(
			_mm_mul_ps(w1, q2),
			_mm_mul_ps(w2, q1)
		);

		//cross v1, v2
		//__m128 v1_yzx = simd::Swizzle<1, 2, 0, 3>(q1);
		//__m128 v2_zxy = simd::Swizzle<2, 0, 1, 3>(q2);
		//__m128 v1_zxy = simd::Swizzle<2, 0, 1, 3>(q1);
		//__m128 v2_yzx = simd::Swizzle<1, 2, 0, 3>(q2);

		__m128 v1_yzx = simd::Swizzle<kAxisY, kAxisZ, kAxisX, kAxisW>(q1);
		__m128 v2_zxy = simd::Swizzle<kAxisZ, kAxisX, kAxisY, kAxisW>(q2);
		__m128 v1_zxy = simd::Swizzle<kAxisZ, kAxisX, kAxisY, kAxisW>(q1);
		__m128 v2_yzx = simd::Swizzle<kAxisY, kAxisZ, kAxisX, kAxisW>(q2);

		__m128 cross = _mm_sub_ps(
			_mm_mul_ps(v1_yzx, v2_zxy),
			_mm_mul_ps(v1_zxy, v2_yzx)
		);

		//xyz = w1*v2+w2*v1+cross
		__m128 xyz = _mm_add_ps(v, cross);

		/// w = w1*w2 - dot(v1,v2)
		/// 1 lane 
		/// 0x7f -> 0111 1111 : op first 3 & store 4 (first)
		/// 0x71 -> 0111 0001 : op first 3 & store 1 (first)
		/// 0x78 -> 0111 1000 : op first 3 & store 1 (last)
		__m128 dot = _mm_dp_ps(q1, q2, 0x78);
		__m128 w = _mm_sub_ps(_mm_mul_ps(w1, w2), dot);

		//xyz +w
		mValue = _mm_blend_ps(xyz, w, 0b1000);
		return *this;

#else

		float rx = rhs.X();
		float ry = rhs.Y();
		float rz = rhs.Z();
		float rw = rhs.W();

		mValue = Vec4(
			W() * rx + X() * rw + Y() * rz - Z() * ry,
			W() * ry - X() * rz + Y() * rw + Z() * rx,
			W() * rz + X() * ry - Y() * rx + Z() * rw,
			W() * rw - X() * rx - Y() * ry - Z() * rz
		);

		return *this;
#endif // VX_SIMD_SSE
	}

}