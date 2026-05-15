#version 400


out vec4  FragColour;

in vec3 gColour;

void main()
{
	FragColour = vec4(1.0f, 0.0f, 1.0f, 1.0f);
	FragColour = vec4(gColour, 1.0f);
}