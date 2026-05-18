#include "Vec3.h"
#include "Vec4.h"
#include "Vec2.h"
#include "SimdUtil.h"

namespace vx
{
	inline Vec3::Vec3()
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_setzero_ps();
#else
		for (int i = 0; i < 4; i++)
			mFloats[i] = 0;
#endif // USE_SIMD_SSE
	}
	inline Vec3::Vec3(float x, float y, float z)
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_setr_ps(x, y, z, z);
#else
		mFloats[0] = x;
		mFloats[1] = y;
		mFloats[2] = z;
		mFloats[3] = z;
#endif // USE_SIMD_SSE
	}
	inline Vec3::Vec3(float scalar)
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_set1_ps(scalar);
#else
		for (int i = 0; i < 4; i++)
			mFloats[i] = scalar;
#endif // USE_SIMD_SSE
	}

	inline Vec3::Vec3(const Vec4& rhs) :
		Vec3(rhs.XYZ()) {}

	inline Vec3::Vec3(__m128 vec) : 
		mValue(_mm_shuffle_ps(vec, vec, _MM_SHUFFLE(2, 2, 1, 0)))
	{
	}

	VX_INLINE __m128 Vec3::Value() const
	{
#ifdef VX_SIMD_SSE
		return mValue;
#else
		return _mm_load_ps(mFloats);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE float& Vec3::operator[](uint32_t i)
	{
		VX_ASSERT(i < 3, "Trying to access invalid Vec3 index");
		return mFloats[i];
	}
	inline VX_INLINE float const& Vec3::operator[](uint32_t i) const
	{
		VX_ASSERT(i < 3, "Trying to access invalid Vec3 index");
		return mFloats[i];
	}



	inline VX_INLINE float Vec3::GetLane(const Vec3& v, int idx)
	{
#ifdef VX_SIMD_SSE
		return simd::GetLane(v.mValue, idx);
#else
		return v.mFloats[idx];
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::Zero()
	{
#ifdef VX_SIMD_SSE
		return Vec3(_mm_setzero_ps());
#else
		return Vec3(0.0f);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE void Vec3::ToZero()
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_setzero_ps();
#else
		for (int i = 0; i < 4; i++)
			mFloats[i] = 0.0f;
#endif // VX_SIMD_SSE
	}


	VX_INLINE Vec3 Vec3::Abs() const
	{
#ifdef VX_SIMD_SSE
		return _mm_max_ps(_mm_sub_ps(_mm_setzero_ps(), mValue), mValue);
#else
		return Vec3(std::fabs(mFloats[0]), std::fabs(mFloats[1]), fabs(mFloats[2]));
#endif // USE_SIMD_SSE
	}

	VX_INLINE Vec3 Vec3::Sign() const
	{
#ifdef VX_SIMD_SSE
		return _mm_or_ps(_mm_and_ps(mValue, _mm_set_ps1(-1.0f)), _mm_set_ps1(1.0f));
#else
		return Vec3(std::copysign(1.0f, mFloats[0]), std::copysign(1.0f, mFloats[1]), std::copysign(1.0f, mFloats[2]));
#endif // USE_SIMD_SSE
	}


	VX_INLINE bool Vec3::IsNaN() const
	{
		return (std::isnan(mFloats[0]) || std::isnan(mFloats[1]) || std::isnan(mFloats[2]));
	}


	inline VX_INLINE bool Vec3::IsZero(float eps) const
	{
		//change x + y + z <= eps * eps
		return VxAbs(mFloats[0]) <= eps &&
			VxAbs(mFloats[1]) <= eps &&
			VxAbs(mFloats[2]) <= eps;
	}

	inline VX_INLINE bool Vec3::IsApprox(const Vec3& rhs, float eps_sq) const
	{
		return (rhs - *this).LengthSq() <= eps_sq;
	}

	inline VX_INLINE bool Vec3::IsNormalised(float tolerance) const
	{
		return VxAbs(LengthSq() - 1.0f) <= tolerance;
	}


	inline VX_INLINE Vec3 Vec3::operator+(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		return _mm_add_ps(mValue, rhs.mValue);
#else
		return Vec3(mFloats[0] + rhs.mFloats[0],
			mFloats[1] + rhs.mFloats[1],
			mFloats[2] + rhs.mFloats[2]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3& Vec3::operator+=(const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_add_ps(mValue, rhs.mValue);
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] += rhs.mFloats[i];
#endif // VX_SIMD_SSE

		return *this;
	}

	inline VX_INLINE Vec3 Vec3::operator-(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		return _mm_sub_ps(this->mValue, rhs.mValue);
#else
		return Vec3(mFloats[0] - rhs.mFloats[0],
			mFloats[1] - rhs.mFloats[1],
			mFloats[2] - rhs.mFloats[2]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3& Vec3::operator-=(const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_sub_ps(this->mValue, rhs.mValue);
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] -= rhs.mFloats[i];
#endif // VX_SIMD_SSE

		return *this;
	}

	inline VX_INLINE Vec3 Vec3::operator*(const float scalar) const
	{
#ifdef VX_SIMD_SSE
		//broad cast or load scalar
		return _mm_mul_ps(mValue, _mm_set_ps1(scalar));
#else
		return Vec3(mFloats[0] * scalar,
			mFloats[1] * scalar,
			mFloats[2] * scalar);
#endif // VX_SIMD_SSE
	}

	VX_INLINE Vec3 vx::operator*(const float lhs, const Vec3& rhs)
	{
		return rhs * lhs;
	}

	inline VX_INLINE Vec3& Vec3::operator*=(const float scalar)
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_mul_ps(mValue, _mm_set_ps1(scalar));
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] *= scalar;
#endif // VX_SIMD_SSE

		return *this;
	}

	inline Vec3 Vec3::operator/(const float scalar) const
	{
#ifdef VX_SIMD_SSE
		//return _mm_div_ps(value, _mm_set_ps1(scalar));//<- expensive div per lane
		//return _mm_mul_ps(value, _mm_set_ps1(1.0f /scalar)); <- loss precision due 1/s, most case fastest 

		//one lane division, shuffle x lane across, then mul
		__m128 v = _mm_div_ss(_mm_set_ps1(1.0f), _mm_set_ps1(scalar));
		return _mm_mul_ps(mValue, _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0)));
		//return _mm_mul_ps(mValue, _mm_set_ps1(scalar));
#else
		return Vec3(mFloats[0] / scalar,
			mFloats[1] / scalar,
			mFloats[2] / scalar);
#endif // VX_SIMD_SSE
	}

	inline Vec3& Vec3::operator/=(const float scalar)
	{
#ifdef VX_SIMD_SSE
		//return _mm_div_ps(value, _mm_set_ps1(scalar));//<- expensive div per lane
		//return _mm_mul_ps(value, _mm_set_ps1(1.0f /scalar)); <- loss precision due 1/s, most case fastest 

		//one lane division, shuffle x lane across, then mul
		__m128 v = _mm_div_ss(_mm_set_ps1(1.0f), _mm_set_ps1(scalar));
		mValue = _mm_mul_ps(mValue, _mm_shuffle_ps(v, v, _MM_SHUFFLE(0, 0, 0, 0)));
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] /= scalar;
#endif // VX_SIMD_SSE

		return *this;
	}


	inline VX_INLINE Vec3 Vec3::operator-() const
	{
#ifdef VX_SIMD_SSE
		return _mm_sub_ps(_mm_setzero_ps(), mValue);
#else
		return Vec3(-mFloats[0], -mFloats[1], -mFloats[2]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::operator*(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		//broad cast or load scalar
		return _mm_mul_ps(mValue, rhs.mValue);
#else
		return Vec3(mFloats[0] * rhs.mFloats[0], 
			mFloats[1] * rhs.mFloats[1], 
			mFloats[2] * rhs.mFloats[2]);
#endif // VX_SIMD_SSE
	}


	inline VX_INLINE Vec3& Vec3::operator*=(const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		//broad cast or load scalar
		mValue = _mm_mul_ps(mValue, rhs.mValue);
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] *= rhs.mFloats[i];
		mFloats[3] = mFloats[2];
#endif // VX_SIMD_SSE

		return *this;
	}

	inline VX_INLINE Vec3 Vec3::operator/(const Vec3& rhs) const
	{
		
		VX_ASSERT(VxAbs(rhs.mFloats[2] - rhs.mFloats[3]) < kEpsilon, "W must equal Z (vec3), to prevent zero division");

#ifdef VX_SIMD_SSE
		//return _mm_div_ps(value, _mm_set_ps1(scalar));//<- expensive div per lane
		//return _mm_mul_ps(value, _mm_set_ps1(1.0f /scalar)); <- loss precision due 1/s, most case fastest 

		//one lane division, shuffle x lane across, then mul
		return _mm_div_ps(mValue, rhs.mValue);
#else
		return Vec3(mFloats[0] / rhs.mFloats[0],
			mFloats[1] / rhs.mFloats[1],
			mFloats[2] / rhs.mFloats[2]);
#endif // USE_SIMD_SSE
	}

	inline VX_INLINE Vec3& Vec3::operator/=(const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		//return _mm_div_ps(value, _mm_set_ps1(scalar));//<- expensive div per lane
		//return _mm_mul_ps(value, _mm_set_ps1(1.0f /scalar)); <- loss precision due 1/s, most case fastest 

		//one lane division, shuffle x lane across, then mul
		mValue = _mm_div_ps(mValue, rhs.mValue);
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] /= rhs.mFloats[i];
		mFloats[3] = rhs.mFloats[2];
#endif // USE_SIMD_SSE
		return *this;
	}

	VX_INLINE bool Vec3::operator == (const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		__m128 cmp = _mm_cmpeq_ps(mValue, rhs.mValue);
		int mask = _mm_movemask_ps(cmp);
		return (mask & 0b111) == 0b111; //only x,y,z lane
#else
		return mFloats[0] == rhs.mFloats[0] &&
			mFloats[1] == rhs.mFloats[1] &&
			mFloats[2] == rhs.mFloats[2];

#endif // USE_SIMD_SSE
	}

	inline VX_INLINE float Vec3::MinComponent() const
	{
#ifdef VX_SIMD_SSE
		//__m128 shuf1 = _mm_shuffle_ps(value, value, _MM_SHUFFLE(2, 3, 0, 1)); // y z w x
		//__m128 min1 = _mm_min_ps(value, shuf1); // min(x,y) min(y,z) min(z,w) min(w,x)
		//__m128 shuf2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(1, 0, 3, 2)); // z w x y
		//__m128 min2 = _mm_min_ps(min1, shuf2); // min(min(x,y),min(z,w)) min(min(y,z),min(w,x)) ...
		//return _mm_cvtss_f32(min2); // extract the first element

		__m128 v = mValue;
		v = _mm_min_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1))); //min x & z min z & w
		v = _mm_min_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 0, 3, 2)));
		return _mm_cvtss_f32(v);
#else
		return (X() < Y()) ? ((X() < Z()) ? mFloats[0] : mFloats[2]) : (Y() < Z()) ? mFloats[1] : mFloats[2];
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE float Vec3::MaxComponent() const
	{
#ifdef VX_SIMD_SSE
		//__m128 shuf1 = _mm_shuffle_ps(value, value, _MM_SHUFFLE(2, 3, 0, 1)); // y z w x
		//__m128 max1 = _mm_max_ps(value, shuf1); // max(x,y) max(y,z) max(z,w) max(w,x)
		//__m128 shuf2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(1, 0, 3, 2)); // z w x y
		//__m128 max2 = _mm_max_ps(max1, shuf2); // max(max(x,y),max(z,w)) max(max(y,z),max(w,x)) ...
		//return _mm_cvtss_f32(max2); // extract the first element

		__m128 v = mValue;
		v = _mm_max_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 3, 0, 1))); //min x & z (shuffle 
		v = _mm_max_ps(v, _mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 0, 3, 2)));
		return _mm_cvtss_f32(v);
#else
		return (X() > Y()) ? ((X() > Z()) ? mFloats[0] : mFloats[2]) : (Y() > Z()) ? mFloats[1] : mFloats[2];
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Axis Vec3::MinAxis() const
	{
		//return (mFloats[0] < mFloats[1]) ? ((mFloats[0] < mFloats[2]) ? 0 : 2) : (mFloats[1] < mFloats[2]) ? 1 : 2;
		return (mFloats[0] < mFloats[1]) ? ((mFloats[0] < mFloats[2]) ? Axis::X : Axis::Z) : (mFloats[1] < mFloats[2]) ? Axis::Y : Axis::Z;
	}

	inline VX_INLINE Axis Vec3::MaxAxis() const
	{
		//return (mFloats[0] > mFloats[1]) ? ((mFloats[0] > mFloats[2]) ? 0 : 2) : (mFloats[1] > mFloats[2]) ? 1 : 2;
		return (mFloats[0] > mFloats[1]) ? ((mFloats[0] > mFloats[2]) ? Axis::X : Axis::Z) : (mFloats[1] > mFloats[2]) ? Axis::Y : Axis::Z;
	}


	inline VX_INLINE Vec3 Vec3::Min(const Vec3& lhs, const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		return _mm_min_ps(lhs.mValue, rhs.mValue);
#else
		return Vec3(std::min(lhs.mFloats[0], rhs.mFloats[0]),
			std::min(lhs.mFloats[1], rhs.mFloats[1]),
			std::min(lhs.mFloats[2], rhs.mFloats[2]));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::Max(const Vec3& lhs, const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		return _mm_max_ps(lhs.mValue, rhs.mValue);
#else
		return Vec3(std::max(lhs.mFloats[0], rhs.mFloats[0]),
			std::max(lhs.mFloats[1], rhs.mFloats[1]),
			std::max(lhs.mFloats[2], rhs.mFloats[2]));
#endif // VX_SIMD_SSE

	}

	inline VX_INLINE Vec3 Vec3::Clamp(const Vec3& v, const Vec3& min, const Vec3& max)
	{
		return Max(Min(v, max), min);
	}

	inline VX_INLINE float Vec3::Dot(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		/// 0x71 -> 0111 0001 : op first 3 & store 1 (first)
		/// 0xf1 -> 1111 0001 : op first 4 & store 1 (first)
		/// 
		/// 0x77 -> 0111 0111 : op first 3 & store 3 (first)
		/// 0xff -> 1111 1111 : op first 4 & store 4 (first)
		/// 
		/// 0x7f -> 0111 1111 : op first 3 & store 4 (first)
		/// 
		/// as 0111 0001 
		/// high nibble 0111 [bit 4 - 7] (nibble 1 = 4bits, 0.5bytes) 
		/// low nibble 0001	 [bit 0 - 3]
		/// using with _mm_dp_ps 
		/// high nibble defines, the bits to op on (multply its components) 
		/// 0111 x, y, z, without w 
		/// low nibble defines, the bits to store result
		/// 0001 only x excluding y, z, and w

		//dot product op first 3 & store 1 (x) then extract 1 (0:x)
		return _mm_cvtss_f32(_mm_dp_ps(mValue, rhs.mValue, 0x71));

#else
		float dot = 0.0f;
		for (int i = 0; i < 3; ++i)
			dot += (mFloats[i] * rhs.mFloats[i]);
		return dot;
#endif // VX_SIMD_SSE

	}

	inline VX_INLINE float Vec3::Dot(const Vec3& lhs, const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		/// 0x71 -> 0111 0001 : op first 3 & store 1 (first)
		/// 0xf1 -> 1111 0001 : op first 4 & store 1 (first)
		/// 
		/// 0x77 -> 0111 0111 : op first 3 & store 3 (first)
		/// 0xff -> 1111 1111 : op first 4 & store 4 (first)
		/// 
		/// 0x7f -> 0111 1111 : op first 3 & store 4 (first)
		/// 
		/// as 0111 0001 
		/// high nibble 0111 [bit 4 - 7] (nibble 1 = 4bits, 0.5bytes) 
		/// low nibble 0001	 [bit 0 - 3]
		/// using with _mm_dp_ps 
		/// high nibble defines, the bits to op on (multply its components) 
		/// 0111 x, y, z, without w 
		/// low nibble defines, the bits to store result
		/// 0001 only x excluding y, z, and w

		//dot product op first 3 & store 1 (x) then extract 1 (0:x) 0x71 ->0111 0001
		return _mm_cvtss_f32(_mm_dp_ps(lhs.mValue, rhs.mValue, 0x71));
#else
		float dot = 0.0f;
		for (int i = 0; i < 3; ++i)
			dot += (lhs.mFloats[i] * rhs.mFloats[i]);
		return dot;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE float Vec3::Angle(const Vec3& rhs) const
	{
		float len_sq = LengthSq() * rhs.LengthSq();
		VX_ASSERT(len_sq > kEpsilon, "");

		float r = Dot(rhs) / VxSqrt(len_sq);
		r = VxClamp(r, -1.0f, 1.0f);
		return VxAcos(r);
	}

	inline VX_INLINE float Vec3::CosAngle(const Vec3& rhs) const
	{
		float len_sq = LengthSq() * rhs.LengthSq();
		VX_ASSERT(len_sq > kEpsilon, "");

		return Dot(rhs) / VxSqrt(len_sq);
	}

	inline VX_INLINE Vec3 Vec3::Cross(const Vec3& rhs) const
	{
#ifdef VX_SIMD_SSE
		/// y * z - z * y
		/// z * x - x * z
		/// x * y - y * x
		/// 
		/// 
		/// x, y, z   [3210]lhs / rhs
		/// y, z, x   [3021] shuffle order 
		/// 
		/// vl = lhs * shuffled rhs
		/// vr = rhs * shuffled lhs
		/// 
		/// vl - vr
		/// preserve w all through
		__m128 vl = _mm_shuffle_ps(rhs.mValue, rhs.mValue, _MM_SHUFFLE(3, 0, 2, 1));
		/// shuffled y z x w :- right
		vl = _mm_mul_ps(mValue, vl);
		/// xy yz zx ww  [l first r second] 
		__m128 vr = _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(3, 0, 2, 1));
		/// shuffled y z x w :- left
		vr = _mm_mul_ps(rhs.mValue, vr);
		/// xy yz zx ww  [r first l second] (yx zy zx ww) 
		__m128 r = _mm_sub_ps(vl, vr);
		/// result anitsymmetric xy yz zx ww 
		/// required (yz) (zx) (xy)
		/// shuf  ->  y->x, z->y, x->z  3 0 2 1
		return _mm_shuffle_ps(r, r, _MM_SHUFFLE(3, 0, 2, 1));
#else
		return Vec3((Y() * rhs.Z()) - (rhs.Y() * Z()),
			(rhs.X() * Z()) - (X() * rhs.Z()),
			(X() * rhs.Y()) - (rhs.X() * Y()));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::Cross(const Vec3& lhs, const Vec3& rhs)
	{
#ifdef VX_SIMD_SSE
		/// y * z - z * y
		/// z * x - x * z
		/// x * y - y * x
		/// 
		/// 
		/// x, y, z   [3210]lhs / rhs
		/// y, z, x   [3021] shuffle order 
		/// 
		/// vl = lhs * shuffled rhs
		/// vr = rhs * shuffled lhs
		/// 
		/// vl - vr
		/// preserve w all through
		__m128 vl = _mm_shuffle_ps(rhs.mValue, rhs.mValue, _MM_SHUFFLE(3, 0, 2, 1));
		/// shuffled y z x w :- right
		vl = _mm_mul_ps(lhs.mValue, vl);
		/// xy yz zx ww  [l first r second] 
		__m128 vr = _mm_shuffle_ps(lhs.mValue, lhs.mValue, _MM_SHUFFLE(3, 0, 2, 1));
		/// shuffled y z x w :- left
		vr = _mm_mul_ps(rhs.mValue, vr);
		/// xy yz zx ww  [r first l second] (yx zy zx ww) 
		__m128 r = _mm_sub_ps(vl, vr);
		/// result anitsymmetric xy yz zx ww 
		/// required (yz) (zx) (xy)
		/// shuf  ->  y->x, z->y, x->z  3 0 2 1
		return _mm_shuffle_ps(r, r, _MM_SHUFFLE(3, 0, 2, 1));
#else
		return Vec3((lhs.Y() * rhs.Z()) - (rhs.Y() * lhs.Z()),
			(rhs.X() * lhs.Z()) - (lhs.X() * rhs.Z()),
			(lhs.X() * rhs.Y()) - (rhs.X() * lhs.Y()));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE float Vec3::ScalarTriple(const Vec3& b, const Vec3& c) const
	{
		//return GetAxisX().Dot(GetAxisY().Cross(GetAxisZ()));
		// 
		/// scalar equivalent Dot(x, Cross(Y, Z)
		/// TBN
		/// Dot(N, Cross(T, B)
		

		//Cross bc
		__m128 T = _mm_shuffle_ps(c.mValue, c.mValue, _MM_SHUFFLE(3, 0, 2, 1));
		/// shuffled y z x w :- right
		T = _mm_mul_ps(b.mValue, T);
		/// xy yz zx ww  [l first r second] 
		__m128 B = _mm_shuffle_ps(b.mValue, b.mValue, _MM_SHUFFLE(3, 0, 2, 1));
		/// shuffled y z x w :- left
		B = _mm_mul_ps(c.mValue, B);
		/// xy yz zx ww  [r first l second] (yx zy zx ww) 
		T = _mm_sub_ps(T, B);
		/// result anitsymmetric xy yz zx ww 
		/// required (yz) (zx) (xy)
		/// shuf  ->  y->x, z->y, x->z  3 0 2 1
		T = _mm_shuffle_ps(T, T, _MM_SHUFFLE(3, 0, 2, 1));

		/// dot N T
		return _mm_cvtss_f32(_mm_dp_ps(mValue, T, 0x71));


		return mFloats[0] * (b.mFloats[1] * c.mFloats[2] - b.mFloats[2] * c.mFloats[1]) +
			mFloats[1] * (b.mFloats[2] * c.mFloats[0] - b.mFloats[0] * c.mFloats[2]) +
			mFloats[2] * (b.mFloats[0] * c.mFloats[1] - b.mFloats[1] * c.mFloats[0]);
	}

	inline VX_INLINE float Vec3::LengthSq() const
	{
#ifdef VX_SIMD_SSE
		//dot product op first 3 & store 1 (x) then extract 1 (0:x) 0x71 ->0111 0001
		return _mm_cvtss_f32(_mm_dp_ps(mValue, mValue, 0x71));
#else
		float len_sq = 0.0f;
		for (int i = 0; i < 3; ++i)
			len_sq += (mFloats[i] * mFloats[i]);
		return len_sq;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE float Vec3::Length() const
	{
#ifdef VX_SIMD_SSE
		//dot product op first 3 & store 1 (x) then extract 1 (0:x) 0x71 ->0111 0001
		return _mm_cvtss_f32(_mm_sqrt_ss(_mm_dp_ps(mValue, mValue, 0x71)));
#else
		float len_sq = 0.0f;
		for (int i = 0; i < 3; ++i)
			len_sq += (mFloats[i] * mFloats[i]);
		return std::sqrt(len_sq);
#endif // VX_SIMD_SSE
	}



	inline VX_INLINE Vec3 Vec3::Normalised() const
	{
#ifdef VX_SIMD_SSE
		/// 0x7f -> 0111 1111 : op first 3 & store 4 (first)
		///         0zyx dddd 
		__m128 dot = _mm_dp_ps(mValue, mValue, 0x7f);
		__m128 safe_ep = _mm_max_ps(dot, _mm_set_ps1(1e-6f));
		return _mm_div_ps(mValue, _mm_sqrt_ps(safe_ep));
#else
		Vec3 result = *this;
		float length_sq = 0.0f;
		for (int i = 0; i < 3; ++i)
			length_sq += (mFloats[i] * mFloats[i]);
		if (length_sq > 1e-6f)
		{
			const float inv = 1.0f / std::sqrt(length_sq);
			for (int i = 0; i < 3; ++i)
				result.mFloats[i] *= inv;
		}
		return result;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3& Vec3::Normalise()
	{
#ifdef VX_SIMD_SSE
		/// 0x7f -> 0111 1111 : op first 3 & store 4 (first)
		///         0zyx dddd 
		__m128 dot = _mm_dp_ps(mValue, mValue, 0x7f);
		__m128 safe_ep = _mm_max_ps(dot, _mm_set_ps1(1e-6f));
		mValue = _mm_div_ps(mValue, _mm_sqrt_ps(safe_ep));
#else
		float length_sq = 0.0f;
		for (int i = 0; i < 3; ++i)
			length_sq += (mFloats[i] * mFloats[i]);
		if (length_sq > 1e-6f)
		{
			const float inv = 1.0f / std::sqrt(length_sq);
			for (int i = 0; i < 3; ++i)
				mFloats[i] *= inv;
		}
#endif // VX_SIMD_SSE

		return *this;
	}

	inline VX_INLINE Vec3 Vec3::Inverted() const
	{
#ifdef VX_SIMD_SSE
		//return _mm_mul_ps(mValue, _mm_set_ps1(-1.0f)); <-- extra cycle
		return _mm_xor_ps(mValue, _mm_set_ps1(-0.0f)); //<-- 1 cycle (bitwise)
#else
		return Vec3(-mFloats[0], -mFloats[1], -mFloats[2]);
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3& Vec3::Invert()
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_xor_ps(mValue, _mm_set_ps1(-0.0f));
#else
		for (int i = 0; i < 3; ++i)
			mFloats[i] = -mFloats[i];
#endif // VX_SIMD_SSE

		return *this;
	}

	inline VX_INLINE Vec3 Vec3::Perpendicular() const
	{
		if (VxAbs(X()) < VxAbs(Y()))
			return Vec3(0.0f, -Z(), Y());
		else
			return Vec3(-Z(), 0.0f, X());
	}

	inline VX_INLINE Vec3 Vec3::NormalisedPerpendicular() const
	{
#ifdef VX_SIMD_SSE
		if (VxAbs(X()) < VxAbs(Y()))
		{
			__m128 r = _mm_setr_ps(0.0f, -Z(), Y(), Y());
			__m128 d = _mm_dp_ps(r, r, 0x7f);
			__m128 safe_ep = _mm_max_ps(d, _mm_set_ps1(1e-6f));
			r = _mm_div_ps(r, _mm_sqrt_ps(safe_ep));
			return Vec3(r);
		}
		else
		{
			__m128 r = _mm_setr_ps(-Z(), 0.0f, X(), X());
			__m128 d = _mm_dp_ps(r, r, 0x7f);
			__m128 safe_ep = _mm_max_ps(d, _mm_set_ps1(1e-6f));
			r = _mm_div_ps(r, _mm_sqrt_ps(safe_ep));
			return Vec3(r);
		}

#else
		//scalar optimisationm 
		// one div then 2 mul (over 2 div) 
		if (VxAbs(X()) < VxAbs(Y()))
		{ 
			float inv = 1.0f / VxSqrt(Y() * Y() + Z() * Z());
			return Vec3(0.0f, -Z() * inv, Y() * inv);
		}
		else
		{
			float inv = 1.0f / VxSqrt(X() * X() + Z() * Z());
			return Vec3(-Z() * inv, 0.0f, X() * inv);
		}
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::Project(const Vec3& rhs) const
	{
		return Dot(rhs) * rhs;
	}

	inline VX_INLINE Vec3 Vec3::Reject(const Vec3& rhs) const
	{
		return (*this - rhs * Dot(rhs));
	}

	inline VX_INLINE Vec3 Vec3::Reflect(const Vec3& nor) const
	{
		/// R = V - 2 * V.Dot(N) * N
#ifdef VX_SIMD_SSE
		//all lane
		__m128 _2_d = _mm_mul_ps(_mm_set1_ps(2.0f), _mm_dp_ps(mValue, nor.mValue, 0x7f));
		return _mm_mul_ps(_mm_sub_ps(mValue, _2_d), nor.mValue);
#else
		return *this - (2 * Dot(nor)) * nor;
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::Lerp(const Vec3& lhs, const Vec3& rhs, float t)
	{
#ifdef VX_SIMD_SSE
		return simd::Lerp(lhs.mValue, rhs.mValue, t);
#else
		return Vec3(
			VxLerp(lhs.X(), rhs.X(), t),
			VxLerp(lhs.Y(), rhs.Y(), t),
			VxLerp(lhs.Z(), rhs.Z(), t));
#endif // VX_SIMD_SSE

	}

	inline VX_INLINE Vec3 Vec3::Sqrt() const
	{
#ifdef VX_SIMD_SSE
		return Vec3(_mm_sqrt_ps(mValue));
#else
		return Vec3(VxSqrt(mFloats[0]), VxSqrt(mFloats[1]), VxSqrt(mFloats[2]));
#endif // VX_SIMD_SSE
	}

	inline VX_INLINE Vec3& Vec3::SqrtAssign()
	{
#ifdef VX_SIMD_SSE
		mValue = _mm_sqrt_ps(mValue);
#else
		mFloats[0] = VxSqrt(mFloats[0]);
		mFloats[1] = VxSqrt(mFloats[1]);
		mFloats[2] = VxSqrt(mFloats[2]);
#endif // VX_SIMD_SSE

		return *this;
	}


	template<int _X, int _Y, int _Z>
	inline VX_INLINE void vx::Vec3::FlipSignAssign()
	{
		VX_ASSERT(_X == 1 || _X == -1 ||
			_Y == 1 || _Y == -1 ||
			_Z == 1 || _Z == -1, "out of bounds range [-1, 1]");

#ifdef VX_SIMD_SSE
		mValue = _mm_xor_ps(mValue, simd::SignMask<_X, _Y, _Z, _Z>());
#else
		if constexpr (_X < 0) mFloats[0] = -mFloats[0];
		if constexpr (_Y <0) mFloats[1] = -mFloats[1];
		if constexpr (_Z <0) mFloats[2] = -mFloats[2];
		//mFloats[3] = mFloats[2];
#endif // VX_SIMD_SSE
	}

	template<int X, int Y, int Z>
	inline VX_INLINE Vec3 Vec3::FlipSign() const
	{
		Vec3 v(*this);
		v.FlipSignAssign<X, Y, Z>();
		return v;
	}

	template<Axis Swizzle_X, Axis Swizzle_Y, Axis Swizzle_Z>
	inline VX_INLINE [[nodiscard]] Vec3 Vec3::Swizzle() const
	{
		VX_ASSERT(Swizzle_X != Axis::W &&
			Swizzle_Y != Axis::W &&
			Swizzle_Z != Axis::W, "Vec3 swizzle W is invalid");
#ifdef VX_SIMD_SSE
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(static_cast<int>(Swizzle_Z), 
															static_cast<int>(Swizzle_Z), 
															static_cast<int>(Swizzle_Y), 
															static_cast<int>(Swizzle_X)));
#else
		return Vec3(mFloats[static_cast<int>(Swizzle_X)],
				mFloats[static_cast<int>(Swizzle_Y)],
				mFloats[static_cast<int>(Swizzle_Z)]); //?? bug ?? mFloats not mFloat
#endif // VX_SIMD_SSE
	}

	VX_INLINE Vec3 Vec3::Reciprocal() const
	{
		return One() / mValue;
	}

	inline VX_INLINE Vec3 Vec3::Broadcast(float scalar)
	{
#ifdef VX_SIMD_SSE
		return _mm_set_ps1(scalar);
#else
		return Vec4(scalar, scalar, scalar, scalar);
#endif // USE_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::SplatX() const
	{
#ifdef VX_SIMD_SSE
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(0, 0, 0, 0));
#else
		return Vec3(mFloats[0], mFloats[0], mFloats[0]);
#endif // USE_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::SplatY() const
	{
#ifdef VX_SIMD_SSE
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(1, 1, 1, 1));
#else
		return Vec3(mFloats[1], mFloats[1], mFloats[1]);
#endif // USE_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::SplatZ() const
	{
#ifdef VX_SIMD_SSE
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(2, 2, 2, 2));
#else
		return Vec3(mFloats[2], mFloats[2], mFloats[2]);
#endif // USE_SIMD_SSE
	}

	inline VX_INLINE Vec3 Vec3::LoadAligned(const float* v)
	{
#ifdef VX_SIMD_SSE
		return _mm_load_ps(v);
#else
		return Vec3(v[0], v[1], v[2]);
#endif // VX_SIMD_SSE

	}

	inline VX_INLINE Vec3 Vec3::Load(const float* v)
	{
#ifdef VX_SIMD_SSE
		return _mm_loadu_ps(v);
#else
		return Vec3(v[0], v[1], v[2]);
#endif // VX_SIMD_SSE
	}
	inline VX_INLINE Vec4 Vec3::Splat4X() const
	{
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(0, 0, 0, 0));
	}
	inline VX_INLINE Vec4 Vec3::Splat4Y() const
	{
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(1, 1, 1, 1));
	}
	inline VX_INLINE Vec4 Vec3::Splat4Z() const
	{
		return _mm_shuffle_ps(mValue, mValue, _MM_SHUFFLE(2, 2, 2, 2));
	}
}
