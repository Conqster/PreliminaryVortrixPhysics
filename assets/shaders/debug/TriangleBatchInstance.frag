#version 450

#if !RENDERDOC_DEBUG
	#extension GL_ARB_bindless_texture : require
	#extension GL_ARB_gpu_shader_int64 : require
#else
	//fallback texture
	layout(binding = 0) uniform sampler2D uFallback;
	layout(binding = 1) uniform sampler2D uFallbackShadowMap;
#endif

/////OUTs
out vec4 FragColour;

#if !RENDERDOC_DEBUG
	layout(binding = 1) uniform sampler2D uFallbackShadowMap;
#endif
/////INs
in VS_OUT
{
	vec4 vertexColour;
	vec4 colour;
	vec4 fragPosLightSpace;
	vec3 normal;
	vec3 viewPos;
	vec3 fragPos;
	vec2 uv;
#if !RENDERDOC_DEBUG
	flat uint64_t texHandle;
#else
	flat uint texHandle;
 #endif
	flat int flags;
}fs_in;



/////UNIFORMs
//accessing SSBO in fragment shader
//woulf be expensive 
//as all pixel invocated by vert, geo, tess
//would try to access
layout(std140, binding = 2) uniform DirectionalLight
{
  	vec3 direction;
	float ambientIntensity; //0.2f
	vec3 colour;
	float diffuseIntensity; //0.985f
}uLight;


layout(std140, binding = 3) uniform DirectionalShadow
{
	mat4 lightViewProj;
	float far;
#if !RENDERDOC_DEBUG
	uint64_t shadowMap;
#else
	uint shadowMap;
	uint dummy;
 #endif
};



/////FUNCTIONs
float DirShadowCalculation(in sampler2D shadow_map, vec4 shadow_coord);

void main()
{
	//sampler2D tex = instances[fs_in.instanceID].texHandle; //SSBO expensive
	vec3 base_colour = fs_in.colour.rgb;

#if !RENDERDOC_DEBUG
	sampler2D tex = sampler2D(fs_in.texHandle);
	base_colour *= texture(tex, fs_in.uv).rgb;
#else
	base_colour *= texture(uFallback, fs_in.uv).rgb;
#endif

	vec3 light_colour = uLight.colour * 0.7f;
	vec3 ambient = uLight.ambientIntensity * light_colour;// * base_colour; <-- remove colour its already been multply at the end
	

	vec3 L = normalize(-uLight.direction); //surface(fragment) to light
	vec3 N = normalize(fs_in.normal);
	vec3 V = normalize(fs_in.viewPos - fs_in.fragPos);

	float factor = max(dot(N, L), 0.0f);
	vec3 diffuse = light_colour * factor * base_colour;//<-- remove colour its already been multply at the end  0.7f;
	vec3 H = normalize(L + V);
	vec3 specular = uLight.colour * pow(max(dot(N, H), 0.0f), 64.0f);

	///receive shadow bit
#if !RENDERDOC_DEBUG
	sampler2D shadow_map = sampler2D(shadowMap);
	shadow_map = uFallbackShadowMap;
	float shadow = ((fs_in.flags & 2) != 0) ? DirShadowCalculation(shadow_map, fs_in.fragPosLightSpace) : 0.0f;
#else
	float shadow = ((fs_in.flags & 2) != 0) ? DirShadowCalculation(uFallbackShadowMap, fs_in.fragPosLightSpace) : 0.0f;
#endif
	base_colour *= ambient + ((1.0f - shadow) * diffuse + specular);


//	//show speculer 
//	base_colour = vec3(specular);
//	//show normal
////	base_colour = vec3(N);
//	//show absolute normal
	//base_colour = vec3(abs(N));
//	//show frag pos
//	base_colour = vs_out.fragPos;
//	//show frag pos
//	base_colour = vs_out.viewPos;
	//show uv
	//base_colour = vec3(fs_in.uv, 0.0f);
	//base_colour = fs_in.colour.rgb;

	/// gamma correction
	base_colour = pow(base_colour, vec3(1.0f/2.2));
	FragColour = vec4(base_colour, fs_in.colour.a);

	//FragColour = vs_out.colour;
	//FragColour = vec4(normalize(vs_out.normal), 1.0f);
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

float DirShadowCalculation(in sampler2D shadow_map, vec4 shadow_coord)
{
	//project texture coordinate & fecth the center sample
	vec3 p = shadow_coord.xyz / shadow_coord.w;
	
	p = p * 0.5f + 0.5f;

    float shadow = 0.0f;
	float bias = 0.0001f;
	//dynamic bias 
	bias = max(0.05f * (1.0f - dot(normalize(fs_in.normal), normalize(uLight.direction))), 0.005f);
	bias /= far;


	float phi = Random(gl_FragCoord.xy) * 6.283185;

	bool use_PCF = false;
	if(use_PCF)
	{
		//Using PCF
		float s = sin(phi);
		float c = cos(phi);
		mat2 rot = mat2(c, -s, s, c);

		vec2 texel_size = 1.0f / textureSize(shadow_map, 0);
		for(float x = -1.5f; x <= 1.5f; x += 1.0f)
		{
		    for(float y = -1.5f; y <= 1.5f; y += 1.0f)
		    {
				vec2 offset = vec2(x, y);
				offset = rot * offset;
		        float pcf_depth = texture(shadow_map, p.xy + offset * texel_size).r;
		        shadow += (p.z - bias) > pcf_depth ? 1.0f : 0.0f;
		    }
		}
		shadow /= 16.0f;
	}
	else 
	{
		const int sample_count = 16;
		float penumbra_size = 2.0f / textureSize(shadow_map, 0).x;

		for(int i = 0; i < sample_count; i++)
		{
			vec2 offset = VogelDiskSample(i, sample_count, phi);
			float pcf_depth = texture(shadow_map, p.xy + offset * penumbra_size).r;
		    shadow += (p.z - bias) > pcf_depth ? 1.0f : 0.0f;
		}
		shadow /= float(sample_count);
	}

	float fade_start = 0.9f;
	float edge_fade = 1.0f - fade_start;


	vec2 fade_xy = smoothstep(vec2(0.0), vec2(edge_fade), p.xy) * 
					(1.0 - smoothstep(vec2(fade_start), vec2(1.0), p.xy));
	//ignore back
	float fz = 1.0f - smoothstep(fade_start, 1.0f, p.z);

	shadow *= fade_xy.x * fade_xy.y * fz;
	//shadow *= fx * fy * fz;
    return shadow;
}