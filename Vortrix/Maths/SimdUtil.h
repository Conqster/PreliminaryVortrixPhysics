#pragma once
#include "Core.h"
#include "Axis.h"

namespace vx::simd
{

	//constant aliases
	constexpr Axis X = kAxisX; 
	constexpr Axis Y = kAxisY;
	constexpr Axis Z = kAxisZ;
	constexpr Axis W = kAxisW;


	[[nodiscard]] VX_INLINE float GetLane(__m128 v, int idx)
	{

		//idx * 4 byte offset of the float
		const __m128i mask = _mm_set_epi8(
			-1, -1, -1, -1,
			-1, -1, -1, -1,
			-1, -1, -1, -1,
			idx * 4 + 3, idx * 4 + 2, idx * 4 + 1, idx * 4
		);

		__m128 shuf = _mm_castsi128_ps(_mm_shuffle_epi8(_mm_castps_si128(v), mask));
		return _mm_cvtss_f32(shuf);

		//switch (idx)
		//{
		//case 0: return _mm_cvtss_f32(v);
		//case 1: return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(1, 1, 1, 1)));
		//case 2: return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(2, 2, 2, 2)));
		//case 3: return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(3, 3, 3, 3)));
		//default: return _mm_cvtss_f32(v);
		//}
	}

	template<int I>
	[[nodiscard]] VX_INLINE constexpr float GetLane(__m128 v)
	{
		VX_ASSERT(I >= 0 && I < 4, "Lane idx out of ranges [0..3]");
		if constexpr (I == 0)
			return _mm_cvtss_f32(v);
		else
			return _mm_cvtss_f32(_mm_shuffle_ps(v, v, _MM_SHUFFLE(I, I, I, I)));

	}

	template<int X, int Y, int Z, int W>
	VX_INLINE constexpr __m128 SignMask()
	{
		return _mm_castsi128_ps(_mm_set_epi32(
			(W < 0) ? 0x80000000 : 0x00000000,
			(Z < 0) ? 0x80000000 : 0x00000000,
			(Y < 0) ? 0x80000000 : 0x00000000,
			(X < 0) ? 0x80000000 : 0x00000000));
	}

	template<bool X, bool Y, bool Z, bool W>
	VX_INLINE constexpr __m128 LaneMask()
	{
		return _mm_castsi128_ps(_mm_set_epi32(
			W ? -1 : 0,
			Z ? -1 : 0,
			Y ? -1 : 0,
			X ? -1 : 0));
	}

	template<int X, int Y, int Z, int W>
	VX_INLINE constexpr __m128 FlipSign(__m128 v)
	{
		return _mm_xor_ps(v, SignMask<X, Y, Z, W>());
	}

	VX_INLINE __m128 Lerp(__m128 a, __m128 b, float t)
	{
		__m128 tt = _mm_set1_ps(t);
		return _mm_add_ps(_mm_mul_ps(_mm_sub_ps(_mm_set1_ps(1.0f), tt), a),
			_mm_mul_ps(tt, b));
	}
	VX_INLINE __m128 Xor(const __m128& v, const __m128& mask)
	{
		return _mm_xor_ps(v, mask);
	}


	template<Axis Swizzle_X, Axis Swizzle_Y, Axis Swizzle_Z, Axis Swizzle_W>
	VX_INLINE __m128 Swizzle(__m128 v)
	{
		static_assert(X < Axis::W &&
			Y < Axis::W &&
			Z < Axis::W &&
			W <= Axis::W, "Axis out or range");
		//return _mm_shuffle_ps(v, v, _MM_SHUFFLE(W, Z, Y, X));
		return _mm_shuffle_ps(v, v, _MM_SHUFFLE(static_cast<int>(Swizzle_W),
			static_cast<int>(Swizzle_Z),
			static_cast<int>(Swizzle_Y),
			static_cast<int>(Swizzle_X)));
	}

	template<Axis Swizzle_X, Axis Swizzle_Y, Axis Swizzle_Z, Axis Swizzle_W>
	VX_INLINE __m128 Swizzle(__m128 v0, __m128 v1)
	{
		static_assert(X < Axis::W &&
			Y < Axis::W &&
			Z < Axis::W &&
			W <= Axis::W, "Axis out or range");
		//return _mm_shuffle_ps(v, v, _MM_SHUFFLE(W, Z, Y, X));
		return _mm_shuffle_ps(v0, v1, _MM_SHUFFLE(static_cast<int>(Swizzle_W),
			static_cast<int>(Swizzle_Z),
			static_cast<int>(Swizzle_Y),
			static_cast<int>(Swizzle_X)));
	}

} //namespace vx::simd