#version 420

out vec4 FragColour;

in vec4 vColour;

void main()
{
	FragColour = vColour;
	//FragColour = vec4(1.0f, 0.0f, 1.0f, 1.0f);
}