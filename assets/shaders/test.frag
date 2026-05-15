#version 400

struct Light
{
	vec3 direction;
	vec3 colour;
	float intensity;
};

out vec4 FragColour;

in vec3 vColour;
in vec2 vUV;
in vec3 vNormal;
in vec3 vViewPos;
in vec3 vFragPos;
in vec4 vFragPosLightSpace;


uniform sampler2D utextureMap;
uniform sampler2D uShadowMap;
uniform vec4 uColour = vec4(1.0f);
uniform float uShadowFar = 100.0f;

uniform Light ulight;

const float kAmbientStrength = 0.7f;

//Functions
float DirShadowCalculation(vec4 shadow_coord);

void main()
{
	vec3 base_colour = texture(utextureMap, vUV).rgb * uColour.rgb;

	vec3 light_colour = ulight.colour * ulight.intensity;
	
	//vec3 ambient = 0.2f * ulight.colour;
	vec3 ambient = kAmbientStrength * light_colour* base_colour;
	

	vec3 L = normalize(-ulight.direction); //surface(fragment) to light
	vec3 N = normalize(vNormal);
	vec3 V = normalize(vViewPos - vFragPos);
	
	float factor = max(dot(N, L), 0.0f);
//	vec3 diffuse = ulight.colour * factor * 0.985f;//0.7f;
	vec3 diffuse = light_colour * factor * base_colour;//0.7f;
	vec3 H = normalize(L + V);
	vec3 specular = light_colour * pow(max(dot(N, H), 0.0f), 64.0f);// * 0.5f;
	
	float shadow = DirShadowCalculation(vFragPosLightSpace);
	
	base_colour *= ambient + ((1.0f - shadow) * diffuse + specular);


	//base_colour = vec3(specular * 10.0f);
	FragColour = vec4(abs(vColour), 1.0f);
	
	//FragColour = 
	
	FragColour = vec4(abs(N), 1.0f);

	///gamma correction
	base_colour = pow(base_colour, vec3(1.0f/2.2));

	FragColour = vec4(base_colour, uColour.a);
	//FragColour = vec4(vColour, uColour.a);
	//FragColour = vec4(vUV, 0.0f, 1.0f);
	//FragColour = vec4(vFragPos, 1.0f);
	//FragColour = vec4(vViewPos, 1.0f);
}


float Random(vec2 seed)
{
	return fract(sin(dot(seed, vec2(12.9898f, 78.233f))) * 43758.5453123f);
}


// https://www.gamedev.net/tutorials/programming/graphics/contact-hardening-soft-shadows-made-fast-r4906/#:~:text=Figure%203%20actually%20uses%20Vogel,11%20rectangular%20filter%20was%20used.
//Nikos: https://www.4rknova.com/blog/2017/01/01/vogel#:~:text=References%20/%20Further%20Reading-,Theory,follow%20a%20spiral%20pattern%20instead.
vec2 VogelDiskSample(int sample_idx, int sample_count, float phi)
{
	const float golden_angle = 2.39997f; // PI * (3 - sqrt(5))

	float r = sqrt(float(sample_idx) + 0.5f) / sqrt(float(sample_count));
	float theta = float(sample_idx) * golden_angle + phi;

	return r * vec2(cos(theta), sin(theta));
}

float DirShadowCalculation(vec4 shadow_coord)
{
	//project texture coordinate & fecth the center sample
	vec3 p = shadow_coord.xyz / shadow_coord.w;
	
	p = p * 0.5f + 0.5f;

    float shadow = 0.0f;
	float bias = 0.0001f;
	//dynamic bias 
	bias = max(0.05f * (1.0f - dot(normalize(vNormal), normalize(ulight.direction))), 0.005f);
	bias /= uShadowFar;


	float phi = Random(gl_FragCoord.xy) * 6.283185;

	bool use_PCF = false;
	if(use_PCF)
	{
		//Using PCF
		float s = sin(phi);
		float c = cos(phi);
		mat2 rot = mat2(c, -s, s, c);

		vec2 texel_size = 1.0f / textureSize(uShadowMap, 0);
		for(float x = -1.5f; x <= 1.5f; x += 1.0f)
		{
		    for(float y = -1.5f; y <= 1.5f; y += 1.0f)
		    {
				vec2 offset = vec2(x, y);
				offset = rot * offset;
		        float pcf_depth = texture(uShadowMap, p.xy + offset * texel_size).r;
		        shadow += (p.z - bias) > pcf_depth ? 1.0f : 0.0f;
		    }
		}
		shadow /= 16.0f;
	}
	else 
	{
		const int sample_count = 16;
		float penumbra_size = 2.0f / textureSize(uShadowMap, 0).x;

		for(int i = 0; i < sample_count; i++)
		{
			vec2 offset = VogelDiskSample(i, sample_count, phi);
			float pcf_depth = texture(uShadowMap, p.xy + offset * penumbra_size).r;
		    shadow += (p.z - bias) > pcf_depth ? 1.0f : 0.0f;
		}
		shadow /= float(sample_count);
	}

	float fade_start = 0.9f;
	float edge_fade = 1.0f - fade_start;
	//float fx = smoothstep(0.0, edge_fade, p.x) * (1.0 - smoothstep(fade_start, 1.0, p.x));
	//float fy = smoothstep(0.0, edge_fade, p.y) * (1.0 - smoothstep(fade_start, 1.0, p.y));


	vec2 fade_xy = smoothstep(vec2(0.0), vec2(edge_fade), p.xy) * 
					(1.0 - smoothstep(vec2(fade_start), vec2(1.0), p.xy));
	//ignore back
	float fz = 1.0f - smoothstep(fade_start, 1.0f, p.z);

	shadow *= fade_xy.x * fade_xy.y * fz;
	//shadow *= fx * fy * fz;
    return shadow;
}