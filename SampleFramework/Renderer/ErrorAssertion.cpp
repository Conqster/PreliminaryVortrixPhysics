#include "ErrorAssertion.h"
#include <iostream>

void GLClearError(const char* file, int line)
{
	while (glGetError() != GL_NO_ERROR)
	{
		GLenum ErrorCheckValue = glGetError();
		std::cout << "stuck in a while loop \n[FILE]: "<< file << "\n[LINE]:" << line << "\n[GLEW msg]:" << glewGetErrorString(ErrorCheckValue) << "\n";
	}
}

bool GLLogCall(const char* func, const char* file, int line)
{
	bool success = true;
	GLenum err;

	while ((err = glGetError()) != GL_NO_ERROR)
	{
		std::cout << "[OpenGL Error] (" << err << ") in " 
			<< func << " " << file << ":" << line << "\n";
		success = false;
	}
	return success;
}
