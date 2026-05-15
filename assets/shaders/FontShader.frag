#version 330 core 

uniform sampler2D uTexture;

/////INs
in VS_OUT
{
	vec2 uv;
	vec4 colour;
}fs_in;

out vec4 FragColour;



void main()
{

	float alpha = texture(uTexture, fs_in.uv).r;
	if(alpha<0.5f)
		discard;
	vec4 _col = fs_in.colour;
	FragColour = vec4(_col.rgb, alpha);
//	FragColour = vec4(_col.rgb, _col.a*alpha);
}