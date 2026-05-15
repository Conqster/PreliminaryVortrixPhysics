#pragma once

#include "Core/Core.h"
#include "Core/Assertion.h"
#include "ScalarMath.h"

#define VPHX_USE_GLM 0

#define VX_USE_SSE
#define VX_MAT_FULL_MULTIPLY 0

#define VX_VEC_ALIGNMENT sizeof(float) * 4


//
//
//#ifdef VPHX_USE_GLM
//#include <GLM/glm/glm.hpp>
//#endif // VPHX_USE_GLM
//

namespace vx
{
	class Vec3;
	class Mat44;
	class Quat;
}