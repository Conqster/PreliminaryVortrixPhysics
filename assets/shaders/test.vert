#version 400

layout(location = 0) in vec3 pos;
layout(location = 1) in vec3 nor;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 col;

out vec3 vColour;
out vec2 vUV;
out vec3 vNormal;
out vec3 vViewPos;
out vec3 vFragPos;
out vec4 vFragPosLightSpace;

uniform mat4 uModel;
uniform mat4 uProj;
uniform mat4 uView;
uniform mat4 uLightSpaceMat;
uniform vec3 uCameraPos;

void main()
{
	vec4 vert_world_pos = uModel * vec4(pos, 1.0f);

	vFragPos = vert_world_pos.xyz;
	gl_Position = uProj * uView * vert_world_pos;
	vNormal = mat3(transpose(inverse(uModel))) * nor;
	vViewPos = uCameraPos;

	vFragPosLightSpace = uLightSpaceMat * vert_world_pos;
	
	vColour = col;
	vUV = uv;
}
