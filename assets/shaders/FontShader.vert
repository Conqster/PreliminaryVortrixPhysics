#version 330 core 

layout(location = 0) in vec3 vPos;
layout(location = 1) in vec2 vUV;
layout(location = 2) in vec4 vCol;

uniform mat4 uProjection;
uniform mat4 uView;

/////OUTs
out VS_OUT
{
	vec2 uv;
	vec4 colour;
}vs_out;

void main()
{
	gl_Position = uProjection * uView*vec4(vPos, 1.0f);
	vs_out.uv = vUV;
	vs_out.colour = vCol;
}