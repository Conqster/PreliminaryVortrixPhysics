#version 400

//layout(location = 0) in vec2 pos;
//layout(location = 2) in vec2 uv;
//
layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 nor;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 col;


out vec2 vUV;
out vec3 vFragPos;

//out vec3 vViewPos;
//out mat3 vViewMatrix3x3;

uniform mat4 uView;


void main()
{
	gl_Position = vec4(pos.xy * 2.0f, 0.0f, 1.0f);
	vFragPos = vec3(pos.xy * 2.0f, 0.0f);
	vUV = uv;
	
	//vViewPos = uView[3].xyz;
	//vViewMatrix3x3 = mat3(uView);
}