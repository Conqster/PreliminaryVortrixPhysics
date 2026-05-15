#version 400

layout(points) in;
layout(line_strip, max_vertices = 250) out;

const int countPerAxis = 18; ///15*8 120

vec3 forward = vec3(0.0f, 0.0f, 1.0f);
vec3 right = vec3(1.0f, 0.0f, 0.0f);

uniform mat4 uProj;
uniform mat4 uView;
uniform mat4 uModel;

uniform int uLOD;

out vec3 gColour;

void main()
{
	//for forward axis 
	float length = 500.0f;

	
	//offset to determine, next level eg 0 for center 
	//left side -> -5 -4 -3 -2 -1 
	//right side-> +1 +2 +3 +4 +5 
	//this values for left & right; are the the offset value but count multiples
	//value is determined by the unit offset (then mutilped by the count idx).
	//side for (forward): determined by the opposite in this right. and vice verse
	
	mat4 world_space = uProj * uView;
	vec3 pf1 = forward * length;
	vec3 pf2 = -pf1;
	vec3 pr1 = right * length;
	vec3 pr2 = -pr1;
	
		//for vertical lines
	float offset_mag = 1.0f * ((uLOD > 0) ? 10.0f : 1.0f);
	vec3 offset_by = right * offset_mag;
	//draw origin
	gColour = vec3(0.0f, 1.0f, 0.0f);
		
		
		gl_Position = world_space * vec4(pf1, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pf2, 1.0f);
		EmitVertex();
		
		EndPrimitive();
		
		
		gl_Position = world_space * vec4(pf1, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pf2, 1.0f);
		EmitVertex();
		
		EndPrimitive();
		
		//for horizontal
		gColour = vec3(1.0f, 0.0f, 0.0f);
		
	
		gl_Position = world_space * vec4(pr1, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pr2, 1.0f);
		EmitVertex();
		
		EndPrimitive();
		
		gl_Position = world_space * vec4(pr1, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pr2, 1.0f);
		EmitVertex();
		
		EndPrimitive();
	
	gColour = vec3(1.0f, 1.0f, 1.0f);

	for(int i = 1; i < countPerAxis; i++)
	{
		
		
		offset_by = right * offset_mag * i;
		gl_Position = world_space * vec4(pf1 + offset_by, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pf2 + offset_by, 1.0f);
		EmitVertex();
		
		EndPrimitive();
		
		
		gl_Position = world_space * vec4(pf1 - offset_by, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pf2 - offset_by, 1.0f);
		EmitVertex();
		
		EndPrimitive();
		
		//for horizontal
		
		offset_by = forward * offset_mag * i;
		gl_Position = world_space * vec4(pr1 + offset_by, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pr2 + offset_by, 1.0f);
		EmitVertex();
		
		EndPrimitive();
		
		gl_Position = world_space * vec4(pr1 - offset_by, 1.0f);
		EmitVertex();
		gl_Position = world_space * vec4(pr2 - offset_by, 1.0f);
		EmitVertex();
		
		EndPrimitive();
	}
}