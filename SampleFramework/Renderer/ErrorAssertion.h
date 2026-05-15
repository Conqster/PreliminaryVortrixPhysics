#pragma once
#include <GL/glew.h>


#define VX_GPU_DEBUG_DEPRECATED 1
//TODO(Conqster): later use a physcis core logger
#define ASSERT(x) if(!(x)) __debugbreak();
#if VX_GPU_DEBUG_DEPRECATED
#define GLCall(x) \
	do { \
		GLClearError(__FILE__, __LINE__); \
		x; \
		ASSERT(GLLogCall(#x, __FILE__, __LINE__)); \
	} while (0)
#else
#define GLCall(x) x
#endif // VX_GPU_DEBUG_DEPRECATED

void GLClearError(const char* file, int line);
bool GLLogCall(const char* func, const char* file, int line);