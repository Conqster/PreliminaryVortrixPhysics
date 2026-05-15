#version 430


layout (location = 0) in vec3 pos;		
layout (location = 1) in vec3 nor;		
layout (location = 2) in vec4 col; 	

layout(std140, binding = 0) uniform uCameraTransform
{
  mat4 proj;
  mat4 view;
};


out VS_OUT
{
	vec4 colour;
	vec3 normal;
	vec3 viewPos;
	vec3 fragPos;
}vs_out;

void main()
{
	vec4 vert_world_pos = vec4(pos, 1.0f);

	vs_out.fragPos = vert_world_pos.xyz;
	gl_Position = proj * view * vert_world_pos;
	vs_out.colour = col;
	vs_out.normal = nor;
}