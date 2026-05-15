#version 400

layout (location = 0) in vec4 pos;

uniform mat4 uProj;
uniform mat4 uView;
uniform mat4 uModel;

void main()
{
	mat4 VP = uProj * uView;
	gl_Position = VP[3];// * vec4(0.0f); //<-- origin
	//gl_Position = uProj * uView * pos;// * uModel * pos; //<-- origin
	
}