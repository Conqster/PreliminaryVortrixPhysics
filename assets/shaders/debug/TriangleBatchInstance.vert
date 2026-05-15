#version 450

#if !RENDERDOC_DEBUG
	#extension GL_ARB_bindless_texture : require
	#extension GL_ARB_gpu_shader_int64 : require
#endif

/////INs
layout (location = 0) in vec3 pos;		
layout (location = 1) in vec3 nor;		
layout (location = 2) in vec2 uv;		
layout (location = 3) in vec4 col; 	

/////UNIFORMs
layout(std140, binding = 0) uniform uCameraTransform
{
  mat4 proj;
  mat4 view;
  vec3 viewPos;
};

struct PrimitiveInstance
{
  mat4 matrix;
  mat4 invMatrix;
  vec4 colour;
#if !RENDERDOC_DEBUG
  uint64_t texHandle;
#else
   uint dummy0;
   uint dummy1;
 #endif
   int flags;
};

readonly restrict layout(std430, binding = 1) buffer InstanceBuffer
{
	PrimitiveInstance instances[];
};

layout(std140, binding = 3) uniform DirectionalShadow
{
	mat4 lightViewProj;
  	float far;
#if !RENDERDOC_DEBUG
	uint64_t shadowMap;
#else
	uint shadowMap;
	uint dummy;
 #endif
};

/////OUTs
out VS_OUT
{
	vec4 vertexColour;
	vec4 colour;
	vec4 fragPosLightSpace;
	vec3 normal;
	vec3 viewPos;
	vec3 fragPos;
	vec2 uv;
#if !RENDERDOC_DEBUG
	flat uint64_t texHandle;
#else
	flat uint texHandle;
 #endif
	flat int flags;
}vs_out;

void main()
{
	PrimitiveInstance instance = instances[gl_InstanceID];
	vec4 vert_world_pos = instance.matrix * vec4(pos, 1.0f);

	vs_out.fragPos = vert_world_pos.xyz;
	gl_Position = proj * view * vert_world_pos;

	vs_out.viewPos = viewPos;
	vs_out.colour = instance.colour;// * col; //if fragment wants vertex col wc col, perform in frag
	vs_out.vertexColour = col;
	vs_out.normal = mat3(transpose(instance.invMatrix)) * nor;
	vs_out.fragPosLightSpace = lightViewProj * vert_world_pos;
	vs_out.uv = uv;
	vs_out.flags = instance.flags;
	//vs_out.tex = instance.tex;
#if !RENDERDOC_DEBUG
	vs_out.texHandle = instance.texHandle;
#else
	vs_out.texHandle = 0u;
 #endif
}