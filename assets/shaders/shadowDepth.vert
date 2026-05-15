#version 400

//layout (location = 0) in vec4 pos;
layout (location = 0) in vec3 pos;		
layout (location = 1) in vec3 nor;		
layout (location = 2) in vec2 uv;		
layout (location = 3) in vec4 col; 	

uniform mat4 uLightSpaceMat;
uniform mat4 uModel;

void main()
{
    gl_Position = uLightSpaceMat * uModel * vec4(pos, 1.0f);   
}