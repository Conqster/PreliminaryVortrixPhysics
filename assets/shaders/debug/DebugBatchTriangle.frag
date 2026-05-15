#version 430

out vec4 FragColour;

in VS_OUT
{
	vec4 colour;
	vec3 normal;
	vec3 viewPos;
	vec3 fragPos;
}vs_out;

void main()
{
	FragColour = vs_out.colour;
	//FragColour = vec4(normalize(vs_out.normal), 1.0f);
}