#pragma once

#include <string>
#include <unordered_map>
#include <GL/glew.h>

#include <Vortrix/Maths/VortrixMaths.h>

class Shader
{
public:
	Shader() = default;
	bool Create(const std::string_view name,  const std::string_view ver, const std::string_view frag, const std::string_view geo = "", bool force_debug = false);

	void Bind() const;
	static void UnbindProgram();

	void SetUniformMat4(const char* name, const vx::Mat44& matrix);
	void SetUniform1i(const char* name, int value);
	void SetUniform1f(const char* name, float value);
	void SetUniformVec2(const char* name, const vx::Vec2& value);
	void SetUniformVec3(const char* name, const vx::Vec3& value);
	void SetUniformVec4(const char* name, const vx::Vec4& value);

	void Clear();
private:
	unsigned int mID = 0;
	std::string mName = "unk";

	std::vector<std::string> mUniformNameBuffer = {};
	std::unordered_map<std::string_view, GLint> cacheUniformLocations = 
		std::unordered_map<std::string_view, GLint>();

	std::string ReadFile(const std::string_view shader_file, bool remove_ver = false);

	unsigned int CompileShader(GLenum shader_type, const std::string& source, const std::string& debug_define);
	//GLint GetUniformLocation(const char* name);
	GLint GetUniformLocation(std::string_view name);


	void ReflectUniforms();
};