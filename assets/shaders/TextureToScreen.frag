#version 400

out vec4 FragColour;

in vec2 vUV;
in vec3 vFragPos;
uniform sampler2D uTexture;

void main()
{		
	vec4 colour = texture(uTexture, vUV).rgba;

	if(colour.a == 0.0f)
	discard;

	FragColour = vec4(colour);//, 1.0f);
}