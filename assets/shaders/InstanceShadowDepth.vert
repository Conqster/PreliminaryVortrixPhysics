#version 430
#if !RENDERDOC_DEBUG
	#extension GL_ARB_bindless_texture : require
	#extension GL_ARB_gpu_shader_int64 : require
#endif

layout (location = 0) in vec4 pos;

uniform mat4 uLightSpaceMat;

//struct PrimitiveInstance
//{
//  mat4 matrix;
//  mat4 invMatrix;
//  vec4 colour;
//  sampler2D texHandle;
//};

struct PrimitiveInstance
{
  mat4 matrix;
  mat4 invMatrix;
  vec4 colour;
#if RENDERDOC_DEBUG
  sampler2D texHandle;
#else
  uint64_t texHandle;
 #endif
 int flags;
 uint dummy0;
};

readonly restrict layout(std430, binding = 1) buffer InstanceBuffer
{
	PrimitiveInstance instances[];
};

void main()
{
	PrimitiveInstance inst = instances[gl_InstanceID];
	mat4 M = inst.matrix;
    //gl_Position = uLightSpaceMat * M * pos;   

	bool can_cast = (inst.flags & 1) != 0; 
    gl_Position =  can_cast ? (uLightSpaceMat * M * pos) : vec4(0.0f/0.0f);   
}