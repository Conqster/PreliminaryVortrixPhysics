//#version 420
//
//layout (location = 0) in vec3 pos;		
//layout (location = 1) in vec4 colour; 	
//
//layout(std140, binding = 0) uniform uCameraTransform
//{
//  mat4 proj;
//  mat4 view;
//};
//
//out vec4 vColour;
//
//void main()
//{
//	gl_Position = proj * view * vec4(pos, 1.0f);
//	//vColour = colour;
////}
//


#version 420

layout (location =0) in vec3 v0;
layout (location =1) in vec4 c0;
layout (location =2) in vec3 v1;
layout (location =3) in vec4 c1;
layout (location =4) in float thickness;


layout(std140, binding = 0) uniform uCameraTransform
{
  mat4 proj;
  mat4 view;
};

uniform vec2 uResolution;


out vec4 vColour;



void main()
{
	mat4 proj_view = proj * view;
	vec4 p0 = proj_view * vec4(v0, 1.0f);
	vec4 p1 = proj_view * vec4(v1, 1.0f);


	//pers ndc
	vec2 ndc0 = p0.xy/p0.w;
	vec2 ndc1 = p1.xy/p1.w;

	vec2 dir = normalize(ndc1 - ndc0);

	//perp
	vec2 nor = vec2(-dir.y, dir.x);

	vec2 px_ndc = 2.0f/uResolution;

	float half_width = thickness*0.5f;

	vec2 offset = nor * half_width * px_ndc;




	int corner = gl_VertexID % 4;

	vec2 offset0 = offset*p0.w;
	vec2 offset1 = offset*p1.w;

	vec4 final_pos;
	if(corner == 0)
	{
		final_pos = vec4(p0.xy+offset0, p0.z, p0.w);
		vColour = c0;
	}
	else if(corner == 1)
	{
		final_pos = vec4(p0.xy-offset0, p0.z, p0.w);
		vColour = c0;
	}
	else if(corner == 2)
	{
		final_pos = vec4(p1.xy+offset1, p1.z, p1.w);
		vColour = c1;
	}
	else
	{
		final_pos = vec4(p1.xy-offset1, p1.z, p1.w);
		vColour = c1;
	}

	gl_Position = final_pos;
}