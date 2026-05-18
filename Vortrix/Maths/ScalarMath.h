#pragma once
#include "Core.h"

namespace vx
{
	/////////////////////////////////////////////////////
	// Constants
	/////////////////////////////////////////////////////

	static constexpr float kVxPi = 3.14159265358979323846f;
	static constexpr float kVxTau = 6.283185307179586f;
	/// Default epsilon for float comparison
	static constexpr float kEpsilon = 1e-6f;
	/// Default epsilon for double comparison
	static constexpr float kEpsilonD = 1e-12;
	/// Quiet NaN constant (float), explicit signaling for invalid result
	static constexpr float kQuietNaN = std::numeric_limits<float>::quiet_NaN();
	/// Quiet NaN constant (double), explicit signaling for invalid result
	static constexpr double kQuietNaND = std::numeric_limits<double>::quiet_NaN();

	static constexpr float kMaxf = 3.402823466e+38F;
	static constexpr double kMaxd = 1.7976931348623158e+308;
	static constexpr int kMaxi = 2147483647;

	static constexpr float kInf = std::numeric_limits<float>::infinity();

	/// raw max comparisons; NaNs propagates intentionally.
	/// Preferred to expose numerical errors
	/// @return Maximum of a and b 
	template<typename T>
	VX_INLINE T VxMax(T a, T b)
	{
		return a < b ? b : a;
	}
	/// Use raw comparisons; NaNs propagates intentionally.
	/// Preferred to expose numerical errors
	/// @return Mininum of a and b 
	template<typename T>
	VX_INLINE T VxMin(T a, T b)
	{
		return a < b ? a : b;
	}
	/// Clamps a value to given range using raw comparison
	/// NaNs propagates intentionally, does not sanitize inputs.
	/// @return clamped value
	template<typename T>
	VX_INLINE T VxClamp(T v, T _min, T _max)
	{
		return VxMin(VxMax(v, _min), _max);
	}

	/// Clamps a value to given range using raw comparison
	/// NaNs propagates intentionally, does not sanitize inputs.
	/// @return clamped value
	template<typename T>
	VX_INLINE T VxClamp01(T v)
	{
		return VxMin(VxMax(v, static_cast<T>(0.0)), static_cast<T>(1.0));
	}

	/// @returns absolute value float (NaN propagates)
	VX_INLINE float VxAbs(float v)
	{
		return std::fabs(v);
	}

	/// @returns absolute value double (NaN propagates)
	VX_INLINE double VxAbs(double v)
	{
		return std::abs(v);
	}

	template<typename T>
	VX_INLINE T VxAbs(T v)
	{
		return std::fabs(v);
	}
	/// Linear interpolates between a and b by t
	/// No clamping; t is unbounded (NaN propagates) 
	VX_INLINE float VxLerp(float a, float b, float t)
	{
		return ((1.0f - t) * a) + t * b;
	}
	/// @returns square of the value float (NaN propagates)
	VX_INLINE float VxSqr(float v)
	{
		return v * v;
	}
	/// @returns square of the value double (NaN propagates)
	VX_INLINE double VxSqr(double v)
	{
		return v * v;
	}
	/// Square root (float)
	/// Domain: v >= 0
	/// -ive and NaN = NaN
	VX_INLINE float VxSqrt(float v)
	{
		return std::sqrt(v);
	}
	/// Square root (double)
	/// Domain: v >= 0
	/// -ive and NaN = NaN
	VX_INLINE double VxSqrt(double v)
	{
		return std::sqrt(v);
	}

	template<typename T>
	VX_INLINE bool VxIsNaN(T v) { return std::isnan(v); }



	/// Checks whether two floats are approximately equal 
	/// using relative error
	/// NaN inputs returns false.
	/// @param eps Allowed realtive tolerance
	/// @return true, if |a - b| <= eps * max(|a|, |b|, 1)
	VX_INLINE bool VxApprox(float a, float b, float eps = 1e-6f)
	{
		float diff = VxAbs(a - b);
		float scale = VxMax(VxMax(VxAbs(a), VxAbs(b)), 1.0f);
		return diff <= eps * scale;
	}

	/// Checks whether two double are approximately equal 
	/// using relative error
	/// NaN inputs returns false.
	/// @param eps Allowed realtive tolerance
	/// @return true, if |a - b| <= eps * max(|a|, |b|, 1)
	VX_INLINE bool VxApprox(double a, double b, double eps = 1e-6f)
	{
		double diff = VxAbs(a - b);
		double scale = VxMax(VxMax(VxAbs(a), VxAbs(b)), 1.0);
		return diff <= eps * scale;
	}

	VX_INLINE float VxPow(float base, float exp)
	{
		return std::pow(base, exp);
	}

	VX_INLINE double VxPow(double base, double exp)
	{
		return std::pow(base, exp);
	}

	VX_INLINE int VxPow(int base, int exp)
	{
		return std::pow(base, exp);
	}

	//////////////////////////////////////////////////////
	// Trigonometry
	//////////////////////////////////////////////////////

	/// Convert degrees to radians.
	/// @tparam T floating-point scalar types.
	/// @param degrees.
	/// @return Angle in radians.
	template<typename T>
	VX_INLINE T DegToRad(T degrees)
	{
		return degrees * static_cast<T>(kVxPi) / static_cast<T>(180.0f);
	}
	/// Convert radians to degrees.
	/// @tparam T floating-point scalar types.
	/// @param radians.
	/// @return Angle in degrees
	template<typename T>
	VX_INLINE T RadToDeg(T radians)
	{
		return radians * static_cast<T>(180.0f) / static_cast<T>(kVxPi);
	}


	/// Sine function (float).
	/// Uses platform math implementation.
	/// floating-point precision errors apply.
	VX_INLINE float VxSin(float v)
	{
		return std::sin(v);
	}
	/// Sine function (double).
	/// Uses platform math implementation.
	/// floating-point precision errors apply.
	VX_INLINE double VxSin(double v)
	{
		return std::sin(v);
	}
	/// Cosine function (float).
	/// Uses platform math implementation.
	/// floating-point precision errors apply.
	VX_INLINE float VxCos(float v)
	{
		return std::cos(v);
	}
	/// Cosine function (double).
	/// Uses platform math implementation.
	/// floating-point precision errors apply.
	VX_INLINE double VxCos(double v)
	{
		return std::cos(v);
	}

	VX_INLINE float VxTan(float v)
	{
		return std::tan(v);
	}

	/// Arc-cosine function (float).
	/// range [-1, 1]
	/// Out-of-range input produce NaN
	VX_INLINE float VxAcos(float v)
	{
		return std::acos(v);
	}
	/// Arc-cosine function (double).
	/// range [-1, 1]
	/// Out-of-range input produce NaN
	VX_INLINE double VxAcos(double v)
	{
		return std::acos(v);
	}
	/// Arc-sine function (float).
	/// range [-1, 1]
	/// Out-of-range input produce NaN
	VX_INLINE float VxAsin(float v)
	{
		return std::asin(v);
	}
	/// Arc-sine function (double).
	/// range [-1, 1]
	/// Out-of-range input produce NaN
	VX_INLINE double VxAsin(double v)
	{
		return std::asin(v);
	}

	/// Computes the angle (in radians)
	/// from x-axis to the point (x, y)
	/// @return Angle in range [-pi, pi]
	VX_INLINE float VxAtan2(float y, float x)
	{
		return std::atan2(y, x);
	}
}