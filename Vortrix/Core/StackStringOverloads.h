#pragma once
#include "Core.h"
#include "StackString.h"
#include "Maths/Vec3.h"


namespace vx {


	//template<size_t N>
	//VX_INLINE StackString<N>& operator<<(StackString<N>& buff, const Vec3& v)
	//{
	//	char buffer[64];
	//	constexpr const char* format = "{x: %.2f, y: %.2f, z: %.2f}";
	//	int len = snprintf(buffer, sizeof(buffer), format, v[0], v[1], v[2]);
	//	if (len > 0)
	//		buff.Append(buffer);
	//	return buff;
	//}
}