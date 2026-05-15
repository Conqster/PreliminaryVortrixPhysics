#pragma once

#define VX_USE_SSE

#if defined (_MSC_VER)
#define VX_INLINE __forceinline
#else
#define VX_INLINE inline
#endif // defined (_MSVC_VER)

#include <iostream>
#if defined(VX_USE_SSE)
#include <xmmintrin.h>

#endif // defined(USE_SIMD_SSE)

//#define VX_ASSERT(expr, ...) \
//		do { if(!(expr)) { \
//				std::cout << "Assertion Failed (" << #expr << "): \nMessage: " << __VA_ARGS__ << ".\nFile: " << __FILE__ << ".\nLine: " << __LINE__ << ".\n"; \
//				__debugbreak(); \
//		 } } while (0)


//forward declare
namespace vx {
	enum class Axis : std::uint8_t;
	class Vec2;
	class Vec3;
	class Vec4;
	class Mat44;
	class Quat;
}

