#pragma once

#include "Core.h"
#include "Axis.h"
//#include "VxMath.h"



namespace vx
{
	/*
	* Vec4  
	* Primarily maintained to support matrix (Mat44) ops
	* homogenenous coordinated, and SIMD-friendly 
	* 
	* Uses:
	*  - Homogenenous positions (x, y, z, w)
	*  - Directions (w = 0)
	* 
	* Layout: (x, y, z, w)
	*/
	class alignas(16) Vec4
	{
	public:

		Vec4() = default;
		Vec4(float x, float y, float z, float w);
		/// w = z
		Vec4(float x, float y, float z);
		explicit Vec4(float scalar);
		explicit Vec4(const Vec3& vec3);
		Vec4(const Vec3& vec3, float w);
		Vec4(__m128 vec) : mValue(vec) {}

		/// @return x component value
		VX_INLINE float X() const { return mFloats[0]; }
		/// @return y component value
		VX_INLINE float Y() const { return mFloats[1]; }
		/// @return z component value
		VX_INLINE float Z() const { return mFloats[2]; }
		/// @return w component value
		VX_INLINE float W() const { return mFloats[3]; }

		/// set x component
		VX_INLINE void SetX(float v) { mFloats[0] = v; }
		/// set y component
		VX_INLINE void SetY(float v) { mFloats[1] = v; }
		/// set z component
		VX_INLINE void SetZ(float v) { mFloats[2] = v; }
		/// set w component
		VX_INLINE void SetW(float v) { mFloats[3] = v; }

		/// Get SIMD value (const)
		__m128 Value() const;
		/// Get SIMD value (mutable value)
		__m128& Value();

		/// Component accessor by index
		/// @param i Index (0 = x, 1 = y, 2 = z, 3 = w)
		/// @return reference to component
		VX_INLINE float& operator[](uint32_t i);
		/// Component accessor by index const
		/// @param i Index (0 = x, 1 = y, 2 = z, 3 = w)
		/// @return const reference to component
		VX_INLINE float const& operator[](uint32_t i) const;

		/// Read component by simd register lane index
		/// @param v vector
		/// @param idx Lane index [0..3]
		VX_INLINE static float GetLane(const Vec4& v, int idx);

		/// Set both components to zero
		VX_INLINE void ToZero();
		/// Vector with all zero 
		VX_INLINE static Vec4 Zero();
		/// Vector with all one 
		VX_INLINE static Vec4 One() { return Vec4(1.0f); }
		/// vector [1, 0, 0, 0]
		VX_INLINE static Vec4 UnitX() { return Vec4(1.0f, 0.0f, 0.0f, 0.0f); }
		/// vector [0, 1, 0, 0]
		VX_INLINE static Vec4 UnitY() { return Vec4(0.0f, 1.0f, 0.0f, 0.0f); }
		/// vector [0, 0, 1, 0]
		VX_INLINE static Vec4 UnitZ() { return Vec4(0.0f, 0.0f, 1.0f, 0.0f); }
		/// vector [0, 0, 0, 1]
		VX_INLINE static Vec4 UnitW() { return Vec4(0.0f, 0.0f, 0.0f, 1.0f); }

		/// Component-wise absolute value
		VX_INLINE Vec4 Abs() const;
		/// Component-wise sign 
		VX_INLINE Vec4 Sign() const;
		/// Check if any component is NaN
		/// @return true if any component is NaN
		VX_INLINE bool IsNaN() const;
		/// Chech if vector approx zero
		/// @return tolerance allowed absolute error
		VX_INLINE bool IsZero(float tolerance = 1e-6f) const;
		/// Approximate equality check
		/// @return rhs Vector to compare
		/// @param tolerance_sq Square tolerance
		VX_INLINE bool IsApprox(const Vec4& rhs, float tolerance_sq = 1e-12f) const;

		/// operator
		VX_INLINE Vec4 operator+(const Vec4& rhs) const;
		VX_INLINE Vec4& operator+=(const Vec4& rhs);
		VX_INLINE Vec4 operator-(const Vec4& rhs) const;
		VX_INLINE Vec4& operator -=(const Vec4& rhs);
		VX_INLINE Vec4 operator*(const float scalar) const;
		VX_INLINE Vec4& operator*=(const float scalar);
		VX_INLINE Vec4 operator/(const float scalar) const;
		VX_INLINE Vec4& operator/=(const float scalar);
		VX_INLINE Vec4 operator -() const;

		///component wise
		///component wise multiply
		VX_INLINE Vec4 operator*(const Vec4& rhs) const;
		VX_INLINE Vec4& operator*=(const Vec4& rhs);
		/// coponent wise divide
		VX_INLINE Vec4 operator/(const Vec4& rhs) const;
		VX_INLINE Vec4& operator/=(const Vec4& rhs);

		/// Comparison
		VX_INLINE bool operator == (const Vec4& rhs) const;
		VX_INLINE bool operator != (const Vec4& rhs) const { return !(*this == rhs); }

		/// smallest component value
		VX_INLINE float MinComponent() const;
		/// Largest component value
		VX_INLINE float MaxComponent() const;

		/// Index of smallest component
		/// @return 0 for x. 
		/// @return 1 for y.
		/// @return 2 for z.
		VX_INLINE Axis MinAxis() const;
		/// Index of largest component
		/// @return 0 for x. 
		/// @return 1 for y.
		/// @return 2 for z.
		VX_INLINE Axis MaxAxis() const;

		/// @return a vector from the smallest component of lhs & rhs vectors
		VX_INLINE static Vec4 Min(const Vec4& lhs, const Vec4& rhs);
		/// @return a vector from the largest component of lhs & rhs vectors
		VX_INLINE static Vec4 Max(const Vec4& lhs, const Vec4& rhs);
		/// Clamp each component between min & max
		/// @param v in vector
		/// @return v in range [min, max]
		VX_INLINE static Vec4 Clamp(const Vec4& v, const Vec4& min, const Vec4& max);

		/// Dot product
		VX_INLINE float Dot(const Vec4& rhs) const;
		/// Dot product
		VX_INLINE static float Dot(const Vec4& lhs, const Vec4& rhs);

		/// 3D Cross product using xyz components
		/// w component is ingnored
		/// @return (Vec3) Cross of xyz 
		VX_INLINE static Vec3 Cross3(const Vec4& lhs, const Vec4& rhs);

		//@return the squared length (magnitude) of the vector
		VX_INLINE float LengthSq() const;
		///@return the length (magnitude) of the vector
		VX_INLINE float Length() const;

		/// normlise this vector
		/// @return normalise copy of this 
		/// of length 1.
		VX_INLINE Vec4 Normalised() const;
		/// normlise this vector in place
		/// @return reference to this
		VX_INLINE Vec4& Normalise();
		/// Negate Vec4
		/// @return negate copy of this
		VX_INLINE Vec4 Inverted() const;
		/// Negate in place 
		/// @return reference to this
		VX_INLINE Vec4& Invert();

		///Component wise Square root
		VX_INLINE Vec4 Sqrt() const;
		///Component wise Square root in place 
		VX_INLINE Vec4& SqrtAssign();

		/// Flip this vector component sign 
		template<int X, int Y, int Z, int W>
		VX_INLINE void FlipSignAssign();
		/// Flip this vector component sign 
		/// @return this vector flipped
		template<int X, int Y, int Z, int W>
		VX_INLINE Vec4 FlipSign() const;
		//template<int X, int Y, int Z, int W>
		template<Axis Swizzle_X, Axis Swizzle_Y, Axis Swizzle_Z, Axis Swizzle_W>
		VX_INLINE [[nodiscard]] Vec4 Swizzle() const;

		/// Component-wise Reciprocal
		/// @returns a reciprocated vector of this vector (1/this)
		VX_INLINE Vec4 Reciprocal() const;

		/// Extract vec3
		/// @return xyz as Vec3
		VX_INLINE Vec3 XYZ() const;
		/// @return [x, y, z, z]
		VX_INLINE Vec4 XYZZ() const;
		/// @return [x, y, z, 0]
		VX_INLINE Vec4 XYZ0() const;
		/// @return [x, y, z, 1]
		VX_INLINE Vec4 XYZ1() const;

		/// broadcast scalar to all components
		VX_INLINE static Vec4 Broadcast(float scalar);
		/// replicate x component to all lanes
		VX_INLINE Vec4 SplatX() const;
		/// replicate z component to all lanes
		VX_INLINE Vec4 SplatY() const;
		/// replicate z component to all lanes
		VX_INLINE Vec4 SplatZ() const;
		/// replicate w component to all lanes
		VX_INLINE Vec4 SplatW() const;

		/// Load aligned float array
		VX_INLINE Vec4 LoadAligned(const float* v);
		/// Load unaligned float array
		VX_INLINE Vec4 Load(const float* v);


		VX_INLINE void Add3(const Vec3& v3);
		VX_INLINE void Multiply3(const Vec3& v3);

		VX_INLINE std::string ToString() const
		{
			char buffer[64];
			constexpr const char* format = "{x: %.2f, y: %.2f, z: %.2f, w: %.2f}";
			snprintf(buffer, sizeof(buffer), format, mFloats[0], mFloats[1], mFloats[2], mFloats[3]);
			return std::string(buffer);
		}

		VX_INLINE friend std::ostream& operator<<(std::ostream& os, const Vec4& v)
		{
			os << "Vec4(" << v.X() << ", " << v.Y() << ", " << v.Z() << ", " << v.W() << ")";
			return os;
		}

	private:
		union
		{
			float mFloats[4];
			__m128 mValue;
		};
	};
	static_assert(std::is_trivial_v<Vec4>);

}

#include "Vec4.inl"