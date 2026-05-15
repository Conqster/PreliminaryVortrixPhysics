#pragma once
#include "StackString.h"
#include "Maths/Vec3.h"


namespace vx {

	VX_INLINE size_t ToChar(const Vec3& v, char* buff, size_t size) //needs to be in the same namespace
	{
		return snprintf(buff, size, "{x: %.2f, y: %.2f, z: %.2f}", v[0], v[1], v[2]);
	}

	VX_INLINE size_t ToChar(const Vec2& v, char* buff, size_t size) //needs to be in the same namespace
	{
		return snprintf(buff, size, "{x: %.2f, y: %.2f}", v[0], v[1]);
	}

	VX_INLINE size_t ToChar(const Float3& v, char* buff, size_t size) //needs to be in the same namespace
	{
		return snprintf(buff, size, "{x: %.2f, y: %.2f, z: %.2f}", v.x, v.y, v.z);
	}

}