#pragma once
//#include "Vec3.h"
#include "MathCommons.h"


namespace vx
{

	/// Note to self if Float3 is 12 bytes. loading as Vec3 via SIMD
	/// (_mm_loadu_ps) reads 16 bytes
	/// 
	/// enusure this is not the last variable in a heap allocated object 
	/// without at least 4 bytes of "buffer" following it.
	/// 
	/// if float3 sit ar the last 12 bytes of memory page (4KB),
	/// the 16 bytes load hit the next page. if the page is unmapped -> Segfault
	/// 
	/// either pad the end of containing class, or ensure Float3 array
	/// have an extra "dummy" element or padding at the very end.
	/// not the last varable, if pad it 
	/// 
	/// strictly purpose as a read-only structure. 

	struct Float3
	{
		Float3() = default;
		explicit Float3(float v) : x(v), y(v), z(v) {}
		Float3(const Float3& rhs) = default;
		Float3& operator=(const Float3& rhs) = default;
		constexpr Float3(float _x, float _y, float _z) : x(_x), y(_y), z(_z){}

		float& operator[](uint32 i)
		{
			VX_ASSERT(i < 3);
			return (&x)[i];
		}

		const float& operator[](uint32 i) const
		{
			VX_ASSERT(i < 3);
			return (&x)[i];
		}

		bool operator ==(const Float3& rhs) const
		{
			return x == rhs.x &&
					y == rhs.y &&
					z == rhs.z;
		}

		bool operator !=(const Float3& rhs) const { return !(*this == rhs); }

		VX_INLINE std::string ToString() const
		{
			char buffer[64];
			constexpr const char* format = "{x: %.2f, y: %.2f, z: %.2f}";
			snprintf(buffer, sizeof(buffer), format, x, y, z);
			return std::string(buffer);
		}

		VX_INLINE friend std::ostream& operator<<(std::ostream& os, const Float3& v)
		{
			os << "Float3(" << v.x << ", " << v.y << ", " << v.z << ")";
			return os;
		}

		float x, y, z;
	};

	static_assert(std::is_trivial_v<Float3>);
	static_assert(std::is_standard_layout_v<Float3>);
	static_assert(sizeof(Float3) == 12);
	static_assert(alignof(Float3) == alignof(float));
	static_assert(std::is_trivially_copyable_v<Float3>);
}