#include "Mat44.h"
#include "Quat.h"

#include "Vec3.h"

namespace vx
{
	Mat44::Mat44(const float diagonal)
	{
		//mCol[0] = Vec4(diagonal, 0.0f, 0.0f, 0.0f);
		//mCol[1] = Vec4(0.0f, diagonal, 0.0f, 0.0f);
		//mCol[2] = Vec4(0.0f, 0.0f, diagonal, 0.0f);
		//mCol[3] = Vec4(0.0f, 0.0f, 0.0f, diagonal);
		std::fill_n(mFloats, 16, 0.0f);
		mFloats[0] = mFloats[5] =
		mFloats[10] = mFloats[15] = diagonal;
	}

	inline Mat44::Mat44(const Vec4& col0, const Vec4& col1, const Vec4& col2, const Vec4& col3) :
		mCol {col0, col1, col2, col3}
	{}

	inline Mat44::Mat44(const Vec4 & col0, const Vec4 & col1, const Vec4 & col2) :
		mCol {col0, col1, col2, Vec4(0.0f, 0.0f, 0.0f, 1.0f)}
	{}

	inline VX_INLINE Mat44 Mat44::Identity()
	{
		return Mat44(1.0f);
	}

	inline VX_INLINE Mat44 Mat44::Dummy()
	{
		Mat44 result;
		for (size_t i = 0; i < 16; ++i)
			result.mFloats[i] = static_cast<float>(i);
		return result;
	}

	inline VX_INLINE Mat44 Mat44::Translation(const Vec3& v)
	{
		return Mat44(Vec4(1.0f, 0.0f, 0.0f, 0.0f), 
					 Vec4(0.0f, 1.0f, 0.0f, 0.0f),
					 Vec4(0.0f, 0.0f, 1.0f, 0.0f),
					 Vec4(v, 1.0f));
	}
	inline VX_INLINE Mat44 Mat44::Scale(float scale)
	{
		return Mat44(Vec4(scale, 0.0f, 0.0f, 0.0f),
					Vec4(0.0f, scale, 0.0f, 0.0f),
					Vec4(0.0f, 0.0f, scale, 0.0f),
					Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::Scale(const Vec3& scale)
	{
		return Mat44(Vec4(scale.X(), 0.0f, 0.0f, 0.0f),
					 Vec4(0.0f, scale.Y(), 0.0f, 0.0f),
					 Vec4(0.0f, 0.0f, scale.Z(), 0.0f),
					 Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::RotationX(float angle)
	{
		/// | R/S   R   R  Tx || R/s R  R Tx |
		/// |  R   cos -sin Ty ||  R  c  -s Ty |
		/// |  R   sin  cos Tz ||  R  s   c Tz |
		/// |  0    0    0   1 ||  0  0   0  1 |
		/// 
		float s = VxSin(angle);
		float c = VxCos(angle);

		//column major
		return Mat44(Vec4(1.0f, 0.0f, 0.0f, 0.0f),
			Vec4(0.0f, c, s, 0.0f),
			Vec4(0.0f, -s, c, 0.0f),
			Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::RotationY(float angle)
	{
		/// |  cos  R  sin  Tx |
		/// |   R  R/S  R   Ty |
		/// | -sin  R  cos Tz |
		/// |   0   0   0   1 |
		/// 
		float s = VxSin(angle);
		float c = VxCos(angle);

		return Mat44(Vec4(c, 0.0f, -s, 0.0f),
			Vec4(0.0f, 1.0f, 0.0f, 0.0f),
			Vec4(s, 0.0f, c, 0.0f),
			Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::RotationZ(float angle)
	{
		/// | cos -sin   R  Tx |
		/// | sin  cos   R  Ty |
		/// |  R    R   R/S Tz |
		/// |  0    0    0   1 |
		/// 
		float s = VxSin(angle);
		float c = VxCos(angle);

		return Mat44(Vec4(c, s,0.0f, 0.0f),
					 Vec4(-s, c, 0.0f, 0.0f),
					 Vec4(0.0f, 0.0f, 1.0f, 0.0f),
					 Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::Basis(const Vec3& x, const Vec3& y, const Vec3& z)
	{
		return Mat44(Vec4(x, 0.0f),
			Vec4(y, 0.0f),
			Vec4(z, 0.0f),
			Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::BasisTranslation(const Vec3& x, const Vec3& y, const Vec3& z, const Vec3& t)
	{
		return Mat44(Vec4(x, 0.0f),
					Vec4(y, 0.0f),
					Vec4(z, 0.0f),
					Vec4(t, 1.0f));
	}

	inline VX_INLINE Mat44 Mat44::Rotation(const Quat& q)
	{
		////| [1 - 2(yy - zz) xy + wz        xz - wy] | T
		////| [xy - wz        1-2(xx - zz)]  yz + wx] |
		////| [xz + wy        yz - wx        1 - xx - yy] |

		/// assume Quat is unit quaternion 
		/// 
		////| [1 - (yy - zz)	  xy + wz         xz - wy]	 | T
		////| [	  xy - wz      1 - (xx - zz)]     yz + wx]   |
		////| [   xz + wy		  yz - wx      1 - (xx - yy)]|
		VX_ASSERT(q.IsUnitQuat(), "Quaternion is not unit");

#ifdef VX_SIMD_SSE


		__m128 _q = q.XYZW().Value();

		//[tx, ty, tz, tw]
		__m128 t_xyz = _mm_add_ps(_q, _q);
		//[xx, yy, zz, ww]
		__m128 xx_yy_zz_ww = _mm_mul_ps(t_xyz, _q);


		/// diag [yy + zz, xx + zz, xx + yy, 0]
		__m128 diag = _mm_add_ps(simd::Swizzle<1, 0, 0, 3>(xx_yy_zz_ww), simd::Swizzle<2, 2, 1, 3>(xx_yy_zz_ww));
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

		__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<0, 0, 1, 1>(t_xyz), simd::Swizzle<1, 2, 2, 2>(_q));
		__m128 w_zyx = _mm_mul_ps(simd::Swizzle<3, 3, 3, 3>(_q), simd::Swizzle<2, 1, 0, 3>(t_xyz));

		/// require write 
		/// d 01 02
		/// 10 d 12 
		/// 20 21 d
		__m128 r20_01_12 = _mm_add_ps(xy_xz_yz, w_zyx); //01_20_12
		r20_01_12 = simd::Swizzle<1, 0, 2, 2>(r20_01_12);
		__m128 r10_21_02 = _mm_sub_ps(xy_xz_yz, w_zyx); //10_02_21
		r10_21_02 = simd::Swizzle<0, 2, 1, 1>(r10_21_02);

		__m128 xC = _mm_blend_ps(_mm_blend_ps(diag, r20_01_12, 0b1110), r10_21_02, 0b1100);
		__m128 yC = _mm_blend_ps(_mm_blend_ps(r10_21_02, diag, 0b1110), r20_01_12, 0b1100);
		__m128 zC = _mm_blend_ps(_mm_blend_ps(r20_01_12, r10_21_02, 0b1110), diag, 0b1100);
		//ensure affine bottom & translate 0, 0,0 1
		__m128 mask = simd::LaneMask<1, 1, 1, 0>();

		return Mat44(Vec4(_mm_and_ps(xC, mask)),
			Vec4(_mm_and_ps(yC, mask)),
			Vec4(_mm_and_ps(zC, mask)));

#else
		float x = q.X(), y = q.Y(),
			z = q.Z(), w = q.W();
		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		float xx = tx * x;
		float yy = ty * y;
		float zz = tz * z;

		float xy = tx * y;
		float xz = tx * z;
		float xw = tx * w;

		float yz = ty * z;
		float yw = ty * w;
		float zw = tz * w;

		/// basis axes
		return Mat44(Vec4(1.0f - (yy + zz), xy + zw, xz - yw, 0.0f),
			Vec4(xy - zw, (1.0f - zz) - xx, yz + xw, 0.0f),
			Vec4(xz + yw, yz - xw, (1.0f - xx) - yy, 0.0f));
#endif // VX_SIMD_SSE
	}

	

	inline VX_INLINE Mat44 Mat44::RotationTranslation(const Quat& q, const Vec3& t)
	{
		////| [1 - 2(yy - zz) xy + wz        xz - wy] | T
////| [xy - wz        1-2(xx - zz)]  yz + wx] |
////| [xz + wy        yz - wx        1 - xx - yy] |

/// assume Quat is unit quaternion 
/// 
////| [1 - (yy - zz)	  xy + wz         xz - wy]	 | T
////| [	  xy - wz      1 - (xx - zz)]     yz + wx]   |
////| [   xz + wy		  yz - wx      1 - (xx - yy)]|
		VX_ASSERT(q.IsUnitQuat(1e-6f), "Quaternion is not unit");

#ifdef VX_SIMD_SSE


		__m128 _q = q.XYZW().Value();

		//[tx, ty, tz, tw]
		__m128 t_xyz = _mm_add_ps(_q, _q);
		//[xx, yy, zz, ww]
		__m128 xx_yy_zz_ww = _mm_mul_ps(t_xyz, _q);


		/// diag [yy + zz, xx + zz, xx + yy, 0]
		__m128 diag = _mm_add_ps(simd::Swizzle<1, 0, 0, 3>(xx_yy_zz_ww), simd::Swizzle<2, 2, 1, 3>(xx_yy_zz_ww));
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

		__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<0, 0, 1, 1>(t_xyz), simd::Swizzle<1, 2, 2, 2>(_q));
		__m128 w_zyx = _mm_mul_ps(simd::Swizzle<3, 3, 3, 3>(_q), simd::Swizzle<2, 1, 0, 3>(t_xyz));

		/// require write 
		/// d 01 02
		/// 10 d 12 
		/// 20 21 d
		__m128 r20_01_12 = _mm_add_ps(xy_xz_yz, w_zyx); //01_20_12
		r20_01_12 = simd::Swizzle<1, 0, 2, 2>(r20_01_12);
		__m128 r10_21_02 = _mm_sub_ps(xy_xz_yz, w_zyx); //10_02_21
		r10_21_02 = simd::Swizzle<0, 2, 1, 1>(r10_21_02);

		__m128 xC = _mm_blend_ps(_mm_blend_ps(diag, r20_01_12, 0b1110), r10_21_02, 0b1100);
		__m128 yC = _mm_blend_ps(_mm_blend_ps(r10_21_02, diag, 0b1110), r20_01_12, 0b1100);
		__m128 zC = _mm_blend_ps(_mm_blend_ps(r20_01_12, r10_21_02, 0b1110), diag, 0b1100);
		//ensure affine bottom & translate 0, 0,0 1
		__m128 mask = simd::LaneMask<1, 1, 1, 0>();


		Mat44 Mrt = Mat44(Vec4(_mm_and_ps(xC, mask)),
			Vec4(_mm_and_ps(yC, mask)),
			Vec4(_mm_and_ps(zC, mask)),
			Vec4(t, 1.0f));

		Mrt.IsOrthonormal(1e-3f);
		return Mrt;

		//return Mat44(Vec4(_mm_and_ps(xC, mask)),
		//		Vec4(_mm_and_ps(yC, mask)),
		//		Vec4(_mm_and_ps(zC, mask)),
		//		Vec4(t, 1.0f));

#else
		float x = q.X(), y = q.Y(),
			z = q.Z(), w = q.W();
		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		float xx = tx * x;
		float yy = ty * y;
		float zz = tz * z;

		float xy = tx * y;
		float xz = tx * z;
		float xw = tx * w;

		float yz = ty * z;
		float yw = ty * w;
		float zw = tz * w;

		/// basis axes
		return Mat44(Vec4(1.0f - (yy + zz), xy + zw, xz - yw, 0.0f),
			Vec4(xy - zw, (1.0f - zz) - xx, yz + xw, 0.0f),
			Vec4(xz + yw, yz - xw, (1.0f - xx) - yy, 0.0f),
			Vec4(t, 1.0f));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE float& Mat44::operator()(int row, int column)
	{
		VX_ASSERT(row < 4, "Row index out of bounds [0, 3]");
		VX_ASSERT(column < 4, "Column index out of bounds [0, 3]");

		return mCol[column][row];
	}

	inline VX_INLINE float Mat44::operator()(int row, int column) const
	{
		VX_ASSERT(row < 4, "Row index out of bounds [0, 3]");
		VX_ASSERT(column < 4, "Column index out of bounds [0, 3]");

		return mCol[column][row];
	}


	inline VX_INLINE Vec4 Mat44::GetColumn(int i) const
	{
		VX_ASSERT(i < 4, "Column index out of bounds [0, 3]");
		return mCol[i];
	}

	inline VX_INLINE Vec3 Mat44::GetColumn3(int i) const
	{
		VX_ASSERT(i < 4, "Column index out of bounds [0, 3]");
		return Vec3(mCol[i]);
	}

	inline VX_INLINE void Mat44::SetColumn(int i, const Vec4& rhs)
	{
		VX_ASSERT(i < 4, "Column index out of bounds [0, 3]");
		mCol[i] = rhs;
	}

	inline VX_INLINE void Mat44::SetColumn3(int i, const Vec3& rhs)
	{
		VX_ASSERT(i < 4, "Column index out of bounds [0, 3]");
		mCol[i] = Vec4(rhs, i == 3 ? 1.0f : 0.0f);
	}


	inline VX_INLINE bool vx::Mat44::operator==(const Mat44& rhs) const
	{
		return mCol[0] == rhs.mCol[0] && 
			   mCol[1] == rhs.mCol[1] && 
			   mCol[2] == rhs.mCol[2] &&
			   mCol[3] == rhs.mCol[3];
	}

	inline VX_INLINE float Mat44::Determinant3x3() const
	{
		return GetAxisX().ScalarTriple(GetAxisY(), GetAxisZ());
		/// using geomteric form
		/// scalar triple
		//return GetAxisX().Dot(GetAxisY().Cross(GetAxisZ()));
		// 
		//scalar equivalent Dot(x, Cross(Y, Z)
		return mFloats[0] * (mFloats[5] * mFloats[10] - mFloats[6] * mFloats[9]) -
			mFloats[4] * (mFloats[1] * mFloats[10] - mFloats[2] * mFloats[9]) +
			mFloats[8] * (mFloats[1] * mFloats[6] - mFloats[2] * mFloats[5]);
	}

	inline VX_INLINE int Mat44::GetBasisHandness() const
	{
		const float det = Determinant3x3();
		if (det > kEpsilon) return 1; //Right handed
		if (det < -kEpsilon) return -1; //Left handed
		return 0;					   //Degenerate
	}

	inline VX_INLINE bool Mat44::IsAffine() const
	{
		return mFloats[3] == 0.0f && mFloats[7] == 0.0f && 
			   mFloats[11] == 0.0f && mFloats[15] == 1.0f;
	}

	inline VX_INLINE bool Mat44::IsAffine3x3() const
	{
		return mFloats[3] == 0.0f && mFloats[7] == 0.0f &&
			mFloats[11] == 0.0f;
	}

	inline VX_INLINE bool Mat44::IsOrthonormal(float tolerance) const
	{

		if (Determinant3x3() < 0.0f) 
			return false;

#ifdef VX_SIMD_SSE


		/// ????? 
		/// _mm_cmpgt_ps ???
		/// _mm_movemask_ps

		__m128 x = GetAxisX().Value();
		__m128 y = GetAxisY().Value();
		__m128 z = GetAxisZ().Value();

		const __m128 eps = _mm_set1_ps(tolerance);
		const __m128 neg = _mm_set1_ps(-0.0f);

		//[|x|^2, |y|^2, |z|^2, ?] 
		__m128 len_sq = _mm_dp_ps(x, x, 0x71);
		len_sq = _mm_blend_ps(len_sq, _mm_dp_ps(y, y, 0x72), 0b1110);
		len_sq = _mm_blend_ps(len_sq, _mm_dp_ps(z, z, 0x74), 0b1100);

		//|len_sq - 1|
		__m128 diff = _mm_sub_ps(len_sq, _mm_set1_ps(1.0f));
		diff = _mm_andnot_ps(neg, diff); //abs

		if (_mm_movemask_ps(_mm_cmpgt_ps(diff, eps)) & 0x7)
			return false;


		// perpendicular
		//[ x*y, x*z, y*z ]
		__m128 dots = _mm_dp_ps(x, y, 0x71);
		dots = _mm_blend_ps(dots, _mm_dp_ps(x, z, 0x72), 0b1110);
		dots = _mm_blend_ps(dots, _mm_dp_ps(y, z, 0x74), 0b1100);

		dots = _mm_andnot_ps(neg, dots); //abs

		if (_mm_movemask_ps(_mm_cmpgt_ps(dots, eps)) & 0x7)
			return false;
		
		return true;
#else
		Vec3 x = GetAxisX();
		Vec3 y = GetAxisY();
		Vec3 z = GetAxisZ();

		if (VxAbs(x.LengthSq() - 1.0f) > tolerance)return false;
		if (VxAbs(y.LengthSq() - 1.0f) > tolerance)return false;
		if (VxAbs(z.LengthSq() - 1.0f) > tolerance)return false;

		if (VxAbs(x.Dot(y)) > tolerance)return false;
		if (VxAbs(x.Dot(z)) > tolerance)return false;
		if (VxAbs(y.Dot(z)) > tolerance)return false;

		return true;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Mat44 Mat44::Transposed3x3() const
	{
		Mat44 result(*this);

		//not sure if to touch, bottom row making affine
		//user might choose to leave value as Transposed for full transposing
		std::swap(result.mFloats[1], result.mFloats[4]);
		std::swap(result.mFloats[2], result.mFloats[8]);
		std::swap(result.mFloats[6], result.mFloats[9]);
		return result;
	}

	inline VX_INLINE Mat44 Mat44::Transposed() const
	{
#ifdef VX_SIMD_SSE
		Mat44 result;
		__m128 c0 = mCol[0].Value();
		__m128 c1 = mCol[1].Value();
		__m128 c2 = mCol[2].Value();
		__m128 c3 = mCol[3].Value();

		_MM_TRANSPOSE4_PS(c0, c1, c2, c3);

		result.mCol[0] = c0;
		result.mCol[1] = c1;
		result.mCol[2] = c2;
		result.mCol[3] = c3;

		return result;
#else
		Mat44 result;
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				result.mFloats[c * 4 + r] = mFloats[r * 4 + c];
		return result;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Mat44 Mat44::Inverse3x3() const
	{
		VX_ASSERT(IsAffine3x3(), "Matrix not affine, bottom row of full 4x4 must be [0 0 0]");
		const float det = Determinant3x3();
		//VX_ASSERT(VxAbs(det) > kEpsilon, "Matrix is singular (non-invertible)");
	

#ifdef VX_SIMD_SSE

		const __m128 inv_det = _mm_set1_ps(1.0f / det);

		__m128 c0 = mCol[0].Value();
		__m128 c1 = mCol[1].Value();
		__m128 c2 = mCol[2].Value();

		Mat44 result(1.0f);

		///inverse rotation (Adjugate) (transpose of cofactor matrix) upper left 3x3
		///cross c1 c2
		__m128 a = _mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 0, 2, 1)),
				_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 1, 0, 2))),
			_mm_mul_ps(_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 1, 0, 2)),
				_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 0, 2, 1)))
		);

		__m128 b = _mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 0, 2, 1)),
				_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 1, 0, 2))),
			_mm_mul_ps(_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 1, 0, 2)),
				_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 0, 2, 1)))
		);


		__m128 c = _mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 0, 2, 1)),
				_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 1, 0, 2))),
			_mm_mul_ps(_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 1, 0, 2)),
				_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 0, 2, 1)))
		);


		/// (1/det(M)) * adj(M)
		a = _mm_mul_ps(a, inv_det);
		b = _mm_mul_ps(b, inv_det);
		c = _mm_mul_ps(c, inv_det);

		/// transpose
		result.mFloats[0] = a.m128_f32[0];
		result.mFloats[4] = a.m128_f32[1];
		result.mFloats[8] = a.m128_f32[2];

		result.mFloats[1] = b.m128_f32[0];
		result.mFloats[5] = b.m128_f32[1];
		result.mFloats[9] = b.m128_f32[2];

		result.mFloats[2] = c.m128_f32[0];
		result.mFloats[6] = c.m128_f32[1];
		result.mFloats[10] = c.m128_f32[2];

		return result;
#else

		const float inv_det = 1.0f / det;

		Mat44 result(1.0f); //<-- ensure translate & bottom is affine due diagnoal = 1.0 anf off = 0.0f
		//inverse rotation (Adjugate) (transpose of cofactor matrix) upper left 3x3
		/// | 0 4 8 12 |   | a e h l |
		///	| 1 5 9 13 |   | b f i m |
		///	| 2 6 10 14 |  | c g j n |
		///	| 3 7 11 15 |  | d h k o |
		///
		/// | a e h |
		/// | b f i |
		/// | c g j |
		/// 
		/// a:- fj - gi
		/// b:- ej - gh
		/// c:- ei - fh
		/// 
		
		result.mFloats[0] = (mFloats[5] * mFloats[10] - mFloats[6] * mFloats[9]) * inv_det;
		result.mFloats[1] = (mFloats[2] * mFloats[9] - mFloats[1] * mFloats[10]) * inv_det;
		result.mFloats[2] = (mFloats[1] * mFloats[6] - mFloats[2] * mFloats[5]) * inv_det;

		result.mFloats[4] = (mFloats[6] * mFloats[8] - mFloats[4] * mFloats[10]) * inv_det;
		result.mFloats[5] = (mFloats[0] * mFloats[10] - mFloats[2] * mFloats[8]) * inv_det;
		result.mFloats[6] = (mFloats[2] * mFloats[4] - mFloats[0] * mFloats[6]) * inv_det;

		result.mFloats[8] = (mFloats[4] * mFloats[9] - mFloats[5] * mFloats[8]) * inv_det;
		result.mFloats[9] = (mFloats[1] * mFloats[8] - mFloats[0] * mFloats[9]) * inv_det;
		result.mFloats[10] = (mFloats[0] * mFloats[5] - mFloats[1] * mFloats[4]) * inv_det;


		return result;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Mat44 Mat44::InverseAffine() const
	{
		VX_ASSERT(IsAffine(), "Matrix not affine, bottom row must be [0 0 0 1]");

		const float det3x3 = Determinant3x3();
		VX_ASSERT(VxAbs(det3x3) > kEpsilon, "Matrix is singular (non-invertible)");

#ifdef VX_SIMD_SSE
		const __m128 inv_det = _mm_set1_ps(1.0f / det3x3);

		__m128 c0 = mCol[0].Value();
		__m128 c1 = mCol[1].Value();
		__m128 c2 = mCol[2].Value();

		Mat44 result(1.0f);

		///inverse rotation (Adjugate) (transpose of cofactor matrix) upper left 3x3
		///cross c1 c2
		__m128 a = _mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 0, 2, 1)),
				_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 1, 0, 2))),
			_mm_mul_ps(_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 1, 0, 2)),
				_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 0, 2, 1)))
		);

		__m128 b = _mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 0, 2, 1)),
				_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 1, 0, 2))),
			_mm_mul_ps(_mm_shuffle_ps(c2, c2, _MM_SHUFFLE(3, 1, 0, 2)),
				_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 0, 2, 1)))
		);


		__m128 c = _mm_sub_ps(
			_mm_mul_ps(_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 0, 2, 1)),
				_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 1, 0, 2))),
			_mm_mul_ps(_mm_shuffle_ps(c0, c0, _MM_SHUFFLE(3, 1, 0, 2)),
				_mm_shuffle_ps(c1, c1, _MM_SHUFFLE(3, 0, 2, 1)))
		);


		/// (1/det(M)) * adj(M)
		a = _mm_mul_ps(a, inv_det);
		b = _mm_mul_ps(b, inv_det);
		c = _mm_mul_ps(c, inv_det);

		/// transpose
		result.mFloats[0] = a.m128_f32[0];
		result.mFloats[4] = a.m128_f32[1];
		result.mFloats[8] = a.m128_f32[2];

		result.mFloats[1] = b.m128_f32[0];
		result.mFloats[5] = b.m128_f32[1];
		result.mFloats[9] = b.m128_f32[2];

		result.mFloats[2] = c.m128_f32[0];
		result.mFloats[6] = c.m128_f32[1];
		result.mFloats[10] = c.m128_f32[2];


		/// t'(inverse tranlation upper right (1x3)) = -R-1 * t
		//row 0 == a
		//row 1 == b
		//row 2 == c
		__m128 t = mCol[3].Value();
		/// x: 0x71 -> 0111 0001 : op first 3 & store 1
		__m128 x = _mm_dp_ps(a, t, 0x71);
		/// y: 0x72 -> 0111 0010 : op first 3 & store 2
		__m128 y = _mm_dp_ps(b, t, 0x72);
		/// z: 0x74 -> 0111 0100 : op first 3 & store 3
		__m128 z = _mm_dp_ps(c, t, 0x74);

	
		/// -R^-1 * t = -(R^-1 * t) 
		result.mCol[3] = _mm_sub_ps(_mm_setzero_ps(), _mm_or_ps(x, _mm_or_ps(y, z)));
		result.mFloats[15] = 1.0f;

		return result;
#else
		const float inv_det3 = 1.0f / det3x3;

		Mat44 result(1.0f); //<-- ensure translate & bottom is affine due diagnoal = 1.0 anf off = 0.0f
		//inverse rotation (Adjugate) (transpose of cofactor matrix) upper left 3x3
		/// | 0 4 8 12 |   | a e h l |
		///	| 1 5 9 13 |   | b f i m |
		///	| 2 6 10 14 |  | c g j n |
		///	| 3 7 11 15 |  | d h k o |
		///
		/// | a e h |
		/// | b f i |
		/// | c g j |
		/// 
		/// a:- fj - gi
		/// b:- ej - gh
		/// c:- ei - fh
		/// 

		result.mFloats[0] = (mFloats[5] * mFloats[10] - mFloats[6] * mFloats[9]) * inv_det3;
		result.mFloats[1] = (mFloats[2] * mFloats[9] - mFloats[1] * mFloats[10]) * inv_det3;
		result.mFloats[2] = (mFloats[1] * mFloats[6] - mFloats[2] * mFloats[5]) * inv_det3;

		result.mFloats[4] = (mFloats[6] * mFloats[8] - mFloats[4] * mFloats[10]) * inv_det3;
		result.mFloats[5] = (mFloats[0] * mFloats[10] - mFloats[2] * mFloats[8]) * inv_det3;
		result.mFloats[6] = (mFloats[2] * mFloats[4] - mFloats[0] * mFloats[6]) * inv_det3;

		result.mFloats[8] = (mFloats[4] * mFloats[9] - mFloats[5] * mFloats[8]) * inv_det3;
		result.mFloats[9] = (mFloats[1] * mFloats[8] - mFloats[0] * mFloats[9]) * inv_det3;
		result.mFloats[10] = (mFloats[0] * mFloats[5] - mFloats[1] * mFloats[4]) * inv_det3;

		//inverse tranlation upper right (1x3) -R-1 * T
		result.mFloats[12] = -(result.mFloats[0] * mFloats[12] +
			result.mFloats[4] * mFloats[13] +
			result.mFloats[8] * mFloats[14]);

		result.mFloats[13] = -(result.mFloats[1] * mFloats[12] +
			result.mFloats[5] * mFloats[13] +
			result.mFloats[9] * mFloats[14]);

		result.mFloats[14] = -(result.mFloats[2] * mFloats[12] +
			result.mFloats[6] * mFloats[13] +
			result.mFloats[10] * mFloats[14]);

		return result;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Mat44::Multiply3x3(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		/// 0x + 4y + 8z + ;0z
		/// 1x + 5y + 9z + ;0z
		/// 2x + 6y + 10z +;0z
		/// 
		/// option 1: grp matrix basis xyz component
		/// x' = XYZx dot rhs
		/// y' = XYZy dot rhs
		/// z' = XYZz dot rhs
		/// 
		/// basis
		/// X: 012 mul rhs splat x
		/// Y: 456 mul rhs splat y
		/// Z: 8910 mul rhs splat z
		/// 
		/// x'y'z' = X + Y + Z
		/// 
		__m128 v = rhs.Value();
		__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0)));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1))));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2))));
		return Vec3(_mm_shuffle_ps(r, r, _MM_SHUFFLE(2, 2, 1, 0)));
#else
		return Vec3(mFloats[0] * rhs[0] + mFloats[4] * rhs[1] + mFloats[8] * rhs[2],
			mFloats[1] * rhs[0] + mFloats[5] * rhs[1] + mFloats[9] * rhs[2],
			mFloats[2] * rhs[0] + mFloats[6] * rhs[1] + mFloats[10] * rhs[2]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Mat44::Multiply3x3Transposed(const Vec3& rhs) const
	{
		/// 0x + 1y + 2z + ;0z
		/// 4x + 5y + 6z + ;0z
		/// 8x + 9y + 10z +;0z
		/// 
#ifdef VX_SIMD_SSE
		__m128 v = rhs.Value();
		/// 0x71 -> 0111 0001 : op first 3 & store 1 (first)
		/// 0xf1 -> 1111 0001 : op first 4 & store 1 (first)
		/// 
		/// 0x77 -> 0111 0111 : op first 3 & store 3 (first)
		/// 0xff -> 1111 1111 : op first 4 & store 4 (first)
		/// 
		/// 0x7f -> 0111 1111 : op first 3 & store 4 (first)
		/// 
		/// require 
		/// x: 0x71 -> 0111 0001 : op first 3 & store 1
		/// y: 0x72 -> 0111 0010 : op first 3 & store 2
		/// z: 0x7C -> 0111 1100 : op first 3 & store 3&4
		/// 
		/// Each dp_ps writes its result to a unique lane.
		/// OR is used as a lane merge (no overlap) 
		__m128 x = _mm_dp_ps(mCol[0].Value(), v, 0x71);
		__m128 y = _mm_dp_ps(mCol[1].Value(), v, 0x72);
		__m128 zz = _mm_dp_ps(mCol[2].Value(), v, 0x7C);

		__m128 r = _mm_or_ps(x, _mm_or_ps(y, zz));
		return r;
#else
		return Vec3(
			mFloats[0] * rhs.X() + mFloats[1] * rhs.Y() + mFloats[2] * rhs.Z(),
			mFloats[4] * rhs.X() + mFloats[5] * rhs.Y() + mFloats[6] * rhs.Z(),
			mFloats[8] * rhs.X() + mFloats[9] * rhs.Y() + mFloats[10] * rhs.Z());
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Mat44::MultiplyAffine(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		/// 0x + 4y + 8z + ;0z
		/// 1x + 5y + 9z + ;0z
		/// 2x + 6y + 10z +;0z
		/// 
		/// option 1: grp matrix basis xyz component
		/// x' = XYZx dot rhs
		/// y' = XYZy dot rhs
		/// z' = XYZz dot rhs
		/// 
		/// basis
		/// X: 012 mul rhs splat x
		/// Y: 456 mul rhs splat y
		/// Z: 8910 mul rhs splat z
		/// 
		/// x'y'z' = X + Y + Z
		/// 
		__m128 v = rhs.Value();
		__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_set1_ps(rhs.X()));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_set1_ps(rhs.Y())));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_set1_ps(rhs.Z())));
		//translation
		r = _mm_add_ps(r, mCol[3].Value());
		//return Vec3(_mm_shuffle_ps(r, r, _MM_SHUFFLE(2, 2, 1, 0)));
		return Vec3(r);
#else
		return Vec3(mFloats[0] * rhs[0] + mFloats[4] * rhs[1] + mFloats[8] * rhs[2] + mFloats[12],
					mFloats[1] * rhs[0] + mFloats[5] * rhs[1] + mFloats[9] * rhs[2] + mFloats[13],
					mFloats[2] * rhs[0] + mFloats[6] * rhs[1] + mFloats[10] * rhs[2] + mFloats[14]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Mat44 Mat44::Multiply3x3(const Mat44& rhs) const
	{
		Mat44 result(1.0f); //ensures translate & bottom entries are 0, 0, 0, 1
		//with diagonal entries as 1 and off-diagonal as 0's
#ifdef VX_SIMD_SSE

		/// .SetColumn3 & 


		//* | R00 R01 R02 Tx  | | 0 4 8 12 |
		//	*| R10 R11 R12 Ty | | 1 5 9 13 |
		//	*| R20 R21 R23 Tz | | 2 6 10 14 |
		//	*| 0   0   0   1  | | 3 7 11 15 |

		/// first rhs column [0 1 2] T
		///
		/// writign to the first column of result [0 1 2] T 
		/// 
		/// 0:- lhs(this) [0 4 8] dot rhs [0 1 2]T
		/// 1:- lhs(this) [1 5 9] dot rhs [0 1 2]T
		/// 2:- lhs(this) [2 6 10] dot rhs [0 1 2]T
		/// 
		/// secomd rhs column [4 5 6]T
		/// 
		/// 4:- lhs(this) [0 4 8] dot rhs [4 5 6]T
		/// 5:- lhs(this) [1 5 9] dot rhs [4 5 6]T
		/// 6:- lhs(this) [2 6 10] dot rhs[4 5 6]T
		/// 
				
		for (int col = 0; col < 3; ++col)
		{
			__m128 c = rhs.mCol[col].Value();

			__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 0)));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(1, 1, 1, 1))));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 2, 2, 2))));

			result.SetColumn3(col, r);
		}
#else

		for (int col = 0; col < 3; ++col)
		{
			const float r0 = rhs.mFloats[col * 4 + 0];
			const float r1 = rhs.mFloats[col * 4 + 1];
			const float r2 = rhs.mFloats[col * 4 + 2];

			result.mFloats[col * 4 + 0] = mFloats[0] * r0 + mFloats[4] * r1 + mFloats[8] * r2;
			result.mFloats[col * 4 + 1] = mFloats[1] * r0 + mFloats[5] * r1 + mFloats[9] * r2;
			result.mFloats[col * 4 + 2] = mFloats[2] * r0 + mFloats[6] * r1 + mFloats[10] * r2;
		}
#endif // VX_SIMD_SSE
		return result;
	}

	inline VX_INLINE Mat44 Mat44::Multiply3x3LeftTransposed(const Mat44& rhs) const
	{
		Mat44 result(1.0f); //ensures translate & bottom entries are 0, 0, 0, 1
		//with diagonal entries as 1 and off-diagonal as 0's
#ifdef VX_SIMD_SSE

		/// .SetColumn3 & 


		//* | R00 R01 R02 Tx  | | 0 4 8 12 |
		//	*| R10 R11 R12 Ty | | 1 5 9 13 |
		//	*| R20 R21 R23 Tz | | 2 6 10 14 |
		//	*| 0   0   0   1  | | 3 7 11 15 |

		/// first rhs column [0 1 2] T
		///
		/// writign to the first column of result [0 1 2] T 
		/// 
		/// 0:- lhs(this) [0 1 2] dot rhs [0 1 2]T
		/// 1:- lhs(this) [4 5 6] dot rhs [0 1 2]T
		/// 2:- lhs(this) [8 9 10] dot rhs [0 1 2]T
		/// 
		/// secomd rhs column [4 5 6]T
		/// 
		/// 4:- lhs(this)[0 1 2] dot rhs [4 5 6]T
		/// 5:- lhs(this)[4 5 6] dot rhs [4 5 6]T
		/// 6:- lhs(this)[8 9 10] dot rhs[4 5 6]T
		/// 

		{
			const __m128 lf_col0 = mCol[0].Value();
			const __m128 lf_col1 = mCol[1].Value();
			const __m128 lf_col2 = mCol[2].Value();

			for (int i = 0; i < 3; ++i)
			{
				const __m128 rt_col = rhs.mCol[i].Value();

				__m128 x = _mm_dp_ps(lf_col0, rt_col, 0x71);
				__m128 y = _mm_dp_ps(lf_col1, rt_col, 0x72);
				__m128 z = _mm_dp_ps(lf_col2, rt_col, 0x74); //w, needs to be 0

				__m128 r = _mm_or_ps(x, _mm_or_ps(y, z));

				result.SetColumn(i, r);
			}
		}

#else

		for (int i = 0; i < 3; ++i)
		{
			const float c0 = rhs.mFloats[i * 4 + 0];
			const float c1 = rhs.mFloats[i * 4 + 1];
			const float c2 = rhs.mFloats[i * 4 + 2];

			result.mFloats[i * 4 + 0] = mFloats[0] * c0 + mFloats[1] * c1 + mFloats[2] * c2;
			result.mFloats[i * 4 + 1] = mFloats[4] * c0 + mFloats[5] * c1 + mFloats[6] * c2;
			result.mFloats[i * 4 + 2] = mFloats[8] * c0 + mFloats[9] * c1 + mFloats[10] * c2;
		}
#endif // VX_SIMD_SSE
		return result;
	}

	inline VX_INLINE Mat44 Mat44::Multiply3x3RightTransposed(const Mat44& rhs) const
	{
		Mat44 result(1.0f); //ensures translate & bottom entries are 0, 0, 0, 1
		//with diagonal entries as 1 and off-diagonal as 0's
#ifdef VX_SIMD_SSE

		/// .SetColumn3 & 


		//* | R00 R01 R02 Tx  | | 0 4 8 12 |
		//	*| R10 R11 R12 Ty | | 1 5 9 13 |
		//	*| R20 R21 R23 Tz | | 2 6 10 14 |
		//	*| 0   0   0   1  | | 3 7 11 15 |

		/// first rhs column [0 1 2] T
		///
		/// writign to the first column of result [0 1 2] T 
		/// 
		/// 0:- lhs(this) [0 4 8] dot rhs [0 4 8]T
		/// 1:- lhs(this) [1 5 9] dot rhs [0 4 8]T
		/// 2:- lhs(this) [2 6 10] dot rhs [0 4 8]T
		/// 
		/// secomd rhs column [4 5 6]T
		/// 
		/// 4:- lhs(this) [0 4 8] dot rhs [1 5 9]T
		/// 5:- lhs(this) [1 5 9] dot rhs [1 5 9]T
		/// 6:- lhs(this) [2 6 10] dot rhs[1 5 9]T
		/// 
		

		//for (int i = 0; i < 3; ++i)
		//{
		//	const float r0 = rhs.mFloats[4 * 0 + i];// 0 1 
		//	const float r1 = rhs.mFloats[4 * 1 + i];// 4 5
		//	const float r2 = rhs.mFloats[4 * 2 + i];// 8 9

		//	result.mFloats[i * 4 + 0] = mFloats[0] * r0 + mFloats[4] * r1 + mFloats[8] * r2;
		//	result.mFloats[i * 4 + 1] = mFloats[1] * r0 + mFloats[5] * r1 + mFloats[9] * r2;
		//	result.mFloats[i * 4 + 2] = mFloats[2] * r0 + mFloats[6] * r1 + mFloats[10] * r2;
		//}

		{
			const __m128 lf_col0 = mCol[0].Value();
			const __m128 lf_col1 = mCol[1].Value();
			const __m128 lf_col2 = mCol[2].Value();

			const __m128 rt_col0 = rhs.mCol[0].Value();
			const __m128 rt_col1 = rhs.mCol[1].Value();
			const __m128 rt_col2 = rhs.mCol[2].Value();


			__m128 r = _mm_mul_ps(lf_col0, _mm_shuffle_ps(rt_col0, rt_col0, _MM_SHUFFLE(0, 0, 0, 0)));
			r = _mm_add_ps(r, _mm_mul_ps(lf_col1, _mm_shuffle_ps(rt_col1, rt_col1, _MM_SHUFFLE(0, 0, 0, 0))));
			r = _mm_add_ps(r, _mm_mul_ps(lf_col2, _mm_shuffle_ps(rt_col2, rt_col2, _MM_SHUFFLE(0, 0, 0, 0))));

			result.SetColumn3(0, r);

			r = _mm_mul_ps(lf_col0, _mm_shuffle_ps(rt_col0, rt_col0, _MM_SHUFFLE(1, 1, 1, 1)));
			r = _mm_add_ps(r, _mm_mul_ps(lf_col1, _mm_shuffle_ps(rt_col1, rt_col1, _MM_SHUFFLE(1, 1, 1, 1))));
			r = _mm_add_ps(r, _mm_mul_ps(lf_col2, _mm_shuffle_ps(rt_col2, rt_col2, _MM_SHUFFLE(1, 1, 1, 1))));

			result.SetColumn3(1, r);

			r = _mm_mul_ps(lf_col0, _mm_shuffle_ps(rt_col0, rt_col0, _MM_SHUFFLE(2, 2, 2, 2)));
			r = _mm_add_ps(r, _mm_mul_ps(lf_col1, _mm_shuffle_ps(rt_col1, rt_col1, _MM_SHUFFLE(2, 2, 2, 2))));
			r = _mm_add_ps(r, _mm_mul_ps(lf_col2, _mm_shuffle_ps(rt_col2, rt_col2, _MM_SHUFFLE(2, 2, 2, 2))));

			result.SetColumn3(2, r);
		}
#else
		for (int i = 0; i < 3; ++i)
		{
			const float r0 = rhs.mFloats[4 * 0 + i];// 0 1 
			const float r1 = rhs.mFloats[4 * 1 + i];// 4 5
			const float r2 = rhs.mFloats[4 * 2 + i];// 8 9

			result.mFloats[i * 4 + 0] = mFloats[0] * r0 + mFloats[4] * r1 + mFloats[8] * r2;
			result.mFloats[i * 4 + 1] = mFloats[1] * r0 + mFloats[5] * r1 + mFloats[9] * r2;
			result.mFloats[i * 4 + 2] = mFloats[2] * r0 + mFloats[6] * r1 + mFloats[10] * r2;
		}
#endif // VX_SIMD_SSE
		return result;
	}

	inline VX_INLINE Mat44 Mat44::Multiply(const Mat44& rhs) const
	{
		/// 0x + 4y + 8z + ;0z
		/// 1x + 5y + 9z + ;0z
		/// 2x + 6y + 10z +;0z
		/// 
		/// a0 = (a0 * b0) + (a4 * b1) + (a8 * b2) + (a12 * b3)
		/// a1 = (a1 * b0) + (a5 * b1) + (a9 * b2) + (a13 * b3)
		/// a2 = (a2 * b0) + (a6 * b1) + (a10 * b2) + (a14 * b3)
		/// a3 = (a3 * b0) + (a7 * b1) + (a11 * b2) + (a15 * b3)
		/// 
		

		Mat44 result;
#ifdef VX_SIMD_SSE
		for (int i = 0; i < 4; ++i)
		{
			__m128 c = rhs.mCol[i].Value();

			__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 0)));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(1, 1, 1, 1))));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 2, 2, 2))));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[3].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 3, 3, 3))));

			result.SetColumn(i, r);
		}

#else
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
			{
				result.mFloats[c * 4 + r] =
					mFloats[0 * 4 + r] * rhs.mFloats[c * 4 + 0] +
					mFloats[1 * 4 + r] * rhs.mFloats[c * 4 + 1] +
					mFloats[2 * 4 + r] * rhs.mFloats[c * 4 + 2] +
					mFloats[3 * 4 + r] * rhs.mFloats[c * 4 + 3];
			}

#endif // VX_SIMD_SSE
		return result;
	}

	inline VX_INLINE Vec4 Mat44::Multiply(const Vec4& rhs) const
	{
#ifdef VX_SIMD_SSE

		__m128 v = rhs.Value();
		__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0)));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1))));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2))));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[3].Value(), _mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 3, 3, 3))));
		return Vec4(r);
#else
		return Vec4(mFloats[0] * rhs[0] + mFloats[4] * rhs[1] + mFloats[8] * rhs[2] + mFloats[12] * rhs[3],
					mFloats[1] * rhs[0] + mFloats[5] * rhs[1] + mFloats[9] * rhs[2] + mFloats[13] * rhs[3],
					mFloats[2] * rhs[0] + mFloats[6] * rhs[1] + mFloats[10] * rhs[2] + mFloats[14] * rhs[3],
					mFloats[3] * rhs[0] + mFloats[7] * rhs[1] + mFloats[11] * rhs[2] + mFloats[15] * rhs[3]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Mat44 Mat44::MultiplyAffine(const Mat44& rhs) const
	{
		VX_ASSERT(IsAffine() && rhs.IsAffine(), "one of the matrices, is not Affine");

		Mat44 result;

		///3x3 
#ifdef VX_SIMD_SSE
		/// .SetColumn3 & 
		/// .SetTranslation 
		/// Preserves Affine [0, 0, 0, 1]
		/// 
		for (int col = 0; col < 3; ++col)
		{
			__m128 c = rhs.mCol[col].Value();

			__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(0, 0, 0, 0)));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(1, 1, 1, 1))));
			r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_shuffle_ps(c, c, _MM_SHUFFLE(2, 2, 2, 2))));

			result.SetColumn3(col, r);
		}

		__m128 t = rhs.mCol[3].Value();
		__m128 r = _mm_mul_ps(mCol[0].Value(), _mm_shuffle_ps(t, t, _MM_SHUFFLE(0, 0, 0, 0)));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[1].Value(), _mm_shuffle_ps(t, t, _MM_SHUFFLE(1, 1, 1, 1))));
		r = _mm_add_ps(r, _mm_mul_ps(mCol[2].Value(), _mm_shuffle_ps(t, t, _MM_SHUFFLE(2, 2, 2, 2))));
		r = _mm_add_ps(r, mCol[3].Value());
		result.SetTranslation(r);

#else
		for (int col = 0; col < 3; ++col)
		{
			const float r0 = rhs.mFloats[col * 4 + 0];
			const float r1 = rhs.mFloats[col * 4 + 1];
			const float r2 = rhs.mFloats[col * 4 + 2];

			result.mFloats[col * 4 + 0] = mFloats[0] * r0 + mFloats[4] * r1 + mFloats[8] * r2;
			result.mFloats[col * 4 + 1] = mFloats[1] * r0 + mFloats[5] * r1 + mFloats[9] * r2;
			result.mFloats[col * 4 + 2] = mFloats[2] * r0 + mFloats[6] * r1 + mFloats[10] * r2;
		}

		result.mFloats[12] =
			mFloats[0] * rhs.mFloats[12] +
			mFloats[4] * rhs.mFloats[13] +
			mFloats[8] * rhs.mFloats[14] + mFloats[12];

		result.mFloats[13] =
			mFloats[1] * rhs.mFloats[12] +
			mFloats[5] * rhs.mFloats[13] +
			mFloats[9] * rhs.mFloats[14] +
			mFloats[13];

		result.mFloats[14] =
			mFloats[2] * rhs.mFloats[12] +
			mFloats[6] * rhs.mFloats[13] +
			mFloats[10] * rhs.mFloats[14] + mFloats[14];

		//bottom row (affine constant) | 3 7 11 15 | -> [0, 0, 0, 1]
		result.mFloats[3] = result.mFloats[7] = result.mFloats[11] = 0;
		result.mFloats[15] = 1.0f;
#endif // VX_SIMD_SSE

		return result;
	}

	inline VX_INLINE Mat44 Mat44::Add(const Mat44& rhs) const
	{
		Mat44 result;
#ifdef VX_SIMD_SSE
		for (int i = 0; i < 4; ++i)
			result.mCol[i] = mCol[i] + rhs.mCol[i];
#else
		for (int i = 0; i < 16; ++i)
			result.mFloats[i] = mFloats[i] + rhs.mFloats[i];
#endif // VX_SIMD_SSE
		return result;
	}

	inline VX_INLINE Mat44 Mat44::AddAffine(const Mat44& rhs) const
	{
		VX_ASSERT(IsAffine3x3() && rhs.IsAffine3x3(), "one/more matrix is not affine");

		/// two paths, if SSE Vec4 does vectorised add
		/// but if not waste extra scalar add ops
		/// so manual add, to minimise waste
		Mat44 result;
#ifdef VX_SIMD_SSE
		for (int i = 0; i < 4; ++i)
			result.mCol[i] = mCol[i] + rhs.mCol[i]; ///<-- using sse under the hood
		//result.mCol[i] = _mm_add_ps(mCol[i].Value(), rhs.mCol[i].Value());
#else
		for (int i = 0; i < 16; ++i)
			result.mFloats[i] = mFloats[i] + rhs.mFloats[i];
#endif // VX_SIMD_SSE

		//ensure bottom [0 0 0 1] is consitent
		//if both affine 0 0 0 would remain 0
		result.mFloats[15] = 1.0f;
		return result;
	}

	inline VX_INLINE Mat44 Mat44::SkewSymmetric3x3(const Vec3& rhs)
	{
		float x = rhs.X();
		float y = rhs.Y();
		float z = rhs.Z();
		return Mat44(
			Vec4(0.0f, z, -y, 0.0f),
			Vec4(-z, 0.0f, x, 0.0f),
			Vec4(y, -x, 0.0f, 0.0f),
			Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	}


	inline VX_INLINE Mat44 Mat44::Decompose(Vec3& o_scale) const
	{
		VX_ASSERT(IsAffine(), "one/more matrix is not affine");

		VX_ASSERT(VxAbs(Determinant3x3()) > kEpsilon, "Matrix is singular (degenerated linear dependant)");

		Vec3 x = GetAxisX();
		Vec3 y = GetAxisY();
		Vec3 z = GetAxisZ();

		/// see https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process
		/// Gram-scmidt orthogonalisation (removing skew) 

		/// orthognalise y and z
		float x_dot_x = VxMax(x.LengthSq(), kEpsilon);
		y -= (x.Dot(y) / x_dot_x) * x;
		z -= (x.Dot(z) / x_dot_x) * x;
		float y_dot_y = VxMax(y.LengthSq(), kEpsilon);
		z -= (y.Dot(z) / y_dot_y) * y;

		float z_dot_z = VxMax(z.LengthSq(), kEpsilon);
		o_scale = Vec3(x_dot_x, y_dot_y, z_dot_z).SqrtAssign();
		
		// ensure result basis is right handed matrix, if not flip the z axis.
		if (x.Cross(y).Dot(z) < 0.0f)
			o_scale.SetZ(-o_scale.Z());

		return Mat44(Vec4(x / o_scale.X(), 0.0f),
					Vec4(y / o_scale.Y(), 0.0f),
					Vec4(z / o_scale.Z(), 0.0f),
					Vec4(GetTranslation(), 1.0f));
	}

	inline VX_INLINE Vec3 Mat44::Transform(const Vec3& vec) const
	{
		return MultiplyAffine(vec);
	}

	inline VX_INLINE Vec3 Mat44::Transform(const Mat44& matrix, const Vec3& translate)
	{
		return matrix.MultiplyAffine(translate);
	}

	inline VX_INLINE Vec3 Mat44::TransformInverse(const Vec3& vector) const
	{
		return Multiply3x3Transposed(vector - GetTranslation());
	}

	inline VX_INLINE Vec3 Mat44::TransformInverse(const Mat44& matrix, const Vec3& translate)
	{
		return matrix.TransformInverse(translate);
	}

	inline VX_INLINE Vec3 Mat44::TransformDirection(const Vec3& vec) const
	{
		return Multiply3x3(vec);
	}

	inline VX_INLINE Vec3 Mat44::TransformInverseDirection(const Vec3& vec) const
	{
		return Multiply3x3Transposed(vec);
	}

	inline VX_INLINE void Mat44::SetRotation(const Quat& q)
	{

		////| [1 - 2(yy - zz) xy + wz        xz - wy] | T
		////| [xy - wz        1-2(xx - zz)]  yz + wx] |
		////| [xz + wy        yz - wx        1 - xx - yy] |

		/// assume Quat is unit quaternion 
		/// 
		////| [1 - (yy - zz)	  xy + wz         xz - wy]	 | T
		////| [	  xy - wz      1 - (xx - zz)]     yz + wx]   |
		////| [   xz + wy		  yz - wx      1 - (xx - yy)]|
		VX_ASSERT(q.IsUnitQuat(), "Quaternion is not unit");

#ifdef VX_SIMD_SSE


		__m128 _q = q.XYZW().Value();

		//[tx, ty, tz, tw]
		__m128 t_xyz = _mm_add_ps(_q, _q);
		//[xx, yy, zz, ww]
		__m128 xx_yy_zz_ww = _mm_mul_ps(t_xyz, _q);


		/// diag [yy + zz, xx + zz, xx + yy, 0]
		__m128 diag = _mm_add_ps(simd::Swizzle<1, 0, 0, 3>(xx_yy_zz_ww), simd::Swizzle<2, 2, 1, 3>(xx_yy_zz_ww));
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

		__m128 xy_xz_yz = _mm_mul_ps(simd::Swizzle<0, 0, 1, 1>(t_xyz), simd::Swizzle<1, 2, 2, 2>(_q));
		__m128 w_zyx = _mm_mul_ps(simd::Swizzle<3, 3, 3, 3>(_q), simd::Swizzle<2, 1, 0, 3>(t_xyz));

		/// require write 
		/// d 01 02
		/// 10 d 12 
		/// 20 21 d
		__m128 r20_01_12 = _mm_add_ps(xy_xz_yz, w_zyx); //01_20_12
		r20_01_12 = simd::Swizzle<1, 0, 2, 2>(r20_01_12);
		__m128 r10_21_02 = _mm_sub_ps(xy_xz_yz, w_zyx); //10_02_21
		r10_21_02 = simd::Swizzle<0, 2, 1, 1>(r10_21_02);

		__m128 xC = _mm_blend_ps(_mm_blend_ps(diag, r20_01_12, 0b1110), r10_21_02, 0b1100);
		__m128 yC = _mm_blend_ps(_mm_blend_ps(r10_21_02, diag, 0b1110), r20_01_12, 0b1100);
		__m128 zC = _mm_blend_ps(_mm_blend_ps(r20_01_12, r10_21_02, 0b1110), diag, 0b1100);
		//ensure affine bottom & translate 0, 0,0 1
		__m128 mask = simd::LaneMask<1, 1, 1, 0>();

		//ensure affine
		mCol[0] = _mm_and_ps(xC, mask);
		mCol[1] = _mm_and_ps(yC, mask);
		mCol[2] = _mm_and_ps(zC, mask);

#else

		float x = q.X(), y = q.Y(),
			z = q.Z(), w = q.W();
		float tx = x + x;
		float ty = y + y;
		float tz = z + z;

		float xx = tx * x;
		float yy = ty * y;
		float zz = tz * z;

		float xy = tx * y;
		float xz = tx * z;
		float xw = tx * w;

		float yz = ty * z;
		float yw = ty * w;
		float zw = tz * w;


		/// basis axes
		mCol[0] = Vec4(1.0f - (yy + zz), xy + zw, xz - yw, 0.0f);
		mCol[1] = Vec4(xy - zw, (1.0f - zz) - xx, yz + xw, 0.0f);
		mCol[2] = Vec4(xz + yw, yz - xw, (1.0f - xx) - yy, 0.0f);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE void Mat44::SetRotationAndTranslation(const Quat& q, const Vec3& pos)
	{
		SetRotation(q);
		SetTranslation(pos);
	}

	inline VX_INLINE Mat44 Mat44::GetRotationMat33() const
	{
		VX_ASSERT(IsAffine3x3(), "Ensure its affine");
		return Mat44(mCol[0], mCol[1], mCol[2]);
	}

	inline VX_INLINE Quat Mat44::GetRotationQuat() const
	{
		VX_ASSERT(IsOrthonormal(), "Mat44 must be orthonormal");


		float m00 = mFloats[0];
		float m11 = mFloats[5];
		float m22 = mFloats[10];

		float trace = m00 + m11 + m22;

		float qx, qy, qz, qw;

		if (trace > kEpsilon)
		{
			float s_factor = VxSqrt(trace + 1.0f);
			float inv_s = 0.5f / s_factor;

			qx = (mFloats[6] - mFloats[9]) * inv_s;
			qy = (mFloats[8] - mFloats[2]) * inv_s;
			qz = (mFloats[1] - mFloats[4]) * inv_s;
			qw = 0.5f * s_factor;
		}
		else if (m00 > m11 && m00 > m22)
		{
			float s_factor = VxSqrt(m00 - m11 - m22 + 1.0f);
			float inv_s = 0.5f / s_factor; 

			qx = 0.5f * s_factor;
			qy = (mFloats[1] + mFloats[4]) * inv_s;
			qz = (mFloats[8] + mFloats[2]) * inv_s;
			qw = (mFloats[6] - mFloats[9]) * inv_s;
		}
		else if (m11 > m22)
		{
			float s_factor = VxSqrt(m11 - m00 - m22 + 1.0f);
			float inv_s = 0.5f / s_factor;

			qx = (mFloats[1] + mFloats[4]) * inv_s;
			qy = 0.5f * s_factor;
			qz = (mFloats[6] + mFloats[9]) * inv_s;
			qw = (mFloats[8] - mFloats[2]) * inv_s;
			
		}
		else
		{
			float s_factor = VxSqrt(m22 - m00 - m11 + 1.0f);
			float inv_s = 0.5f / s_factor;

		
			qx = (mFloats[8] + mFloats[2]) * inv_s;
			qy = (mFloats[6] + mFloats[9]) * inv_s;
			qz = 0.5f * s_factor;
			qw = (mFloats[1] - mFloats[4]) * inv_s;
		}

		return Quat(qx, qy, qz, qw);
	}



	//inline VX_INLINE Mat44 Mat44::ScaledGlobal(const Vec3& scale)
	//{
	//	return Mat44(mCol[0] * scale.X(), mCol[1] * scale.Y(), mCol[2] * scale.Z(), mCol[3]);
	//}

	//inline VX_INLINE Mat44 Mat44::ScaledLocal(const Vec3& scale)
	//{
	//	Vec4 s(scale, 1.0f);
	//	return Mat44(mCol[0] * s, mCol[1] * s, mCol[2] * s, mCol[3] * s);
	//}

	inline VX_INLINE void Mat44::MakeOrthonormal()
	{
		Vec3 X = GetAxisX();
		Vec3 Y = GetAxisY();
		Vec3 Z = GetAxisZ();

		X.Normalise();

		Y = (Y - X * X.Dot(Y)).Normalise();

		Z = X.Cross(Y);

		SetAxisX(X);
		SetAxisY(Y);
		SetAxisZ(Z);
	}

	inline Mat44 Mat44::InverseRotationTranslation(const Quat& q, const Vec3& t)
	{
		Mat44 M = Rotation(q.Conjugated());
		M.SetTranslation(-M.Multiply3x3(t));
		return M;
	}

	inline VX_INLINE Vec4& Mat44::operator[](int column)
	{
		VX_ASSERT(column < 4, "Column index out of bounds [0, 3]");
		return mCol[column];
	}

	inline VX_INLINE Vec4 Mat44::operator[](int column) const
	{
		VX_ASSERT(column < 4, "Column index out of bounds [0, 3]");
		return mCol[column];
	}
}