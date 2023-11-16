#version 450

layout(location = 0) in vec3 in_normal;
layout(location = 1) in vec2 in_uv;
layout(location = 2) in vec3 frag_uvw;
layout(location  = 5) in vec4 in_color;

layout(set = 0, binding = 0) uniform UBO {
    layout(offset = 64) vec3 cam_pos;
} ubo;

layout(push_constant) uniform PCO {
    layout(offset = 64) vec4 base_color;
    vec4 metallic_roughness;
    uint material_flag;
} pco;

// global descriptors
layout(set = 0, binding = 2) uniform samplerCube irradiance_map;
layout(set = 0, binding = 3) uniform samplerCube prefilter_map;
layout(set = 0, binding = 4) uniform sampler2D brdf_map;

// material descriptors
layout(set = 1, binding = 0) uniform sampler2D color_map;
layout(set = 1, binding = 1) uniform sampler2D normal_map;
layout(set = 1, binding = 2) uniform sampler2D ao_map;
layout(set = 1, binding = 3) uniform sampler2D emissive_map;
layout(set = 1, binding = 4) uniform sampler2D metallic_roughness_map;

layout(location = 0) out vec4 out_color;

#define PI 3.1415926535897932384626433832795
const float OCCLUSION_STRENGTH = 0.5f;
const float EMISSIVE_STRENGTH = 1.0f;

const uint BASE_COLOR_TEXTURE_BIT         = 1 << 0;
const uint NORMAL_TEXTURE_BIT             = 1 << 1;
const uint OCCLUSION_TEXTURE_BIT          = 1 << 2;
const uint EMISSIVE_TEXTURE_BIT           = 1 << 3;
const uint METALLIC_ROUGHNESS_TEXTURE_BIT = 1 << 4;

vec3 get_color() {
    if ((pco.material_flag & BASE_COLOR_TEXTURE_BIT) > 0) {
        return texture(color_map, in_uv).rgb;
    }
    return pco.base_color.rgb;
}

vec3 get_normal()
{
    if ((pco.material_flag & NORMAL_TEXTURE_BIT) > 0) {
        vec3 tangentNormal = texture(normal_map, in_uv).xyz * 2.0 - 1.0;

        vec3 Q1  = dFdx(frag_uvw);
        vec3 Q2  = dFdy(frag_uvw);
        vec2 st1 = dFdx(in_uv);
        vec2 st2 = dFdy(in_uv);

        vec3 N   = normalize(in_normal);
        vec3 T  = normalize(Q1*st2.t - Q2*st1.t);
        vec3 B  = -normalize(cross(N, T));
        mat3 TBN = mat3(T, B, N);

        return normalize(TBN * tangentNormal);
    }
    return normalize(in_normal);
}

vec2 get_metallic_roughness() {
    if ((pco.material_flag & METALLIC_ROUGHNESS_TEXTURE_BIT) > 0) {
        return texture(metallic_roughness_map, in_uv).bg;
    }
    return pco.metallic_roughness.bg;
}

vec3 get_emissive() {
    if ((pco.material_flag & EMISSIVE_TEXTURE_BIT) > 0) {
        return texture(emissive_map, in_uv).rgb;
    }
    return vec3(0.0f, 0.0f, 0.0f);
}

// Normal Distribution function --------------------------------------
float D_GGX(float dotNH, float roughness)
{
	float alpha = roughness * roughness;
	float alpha2 = alpha * alpha;
	float denom = dotNH * dotNH * (alpha2 - 1.0) + 1.0;
	return (alpha2)/(PI * denom*denom); 
}

// Geometric Shadowing function --------------------------------------
float G_SchlicksmithGGX(float dotNL, float dotNV, float roughness)
{
	float r = (roughness + 1.0);
	float k = (r*r) / 8.0;
	float GL = dotNL / (dotNL * (1.0 - k) + k);
	float GV = dotNV / (dotNV * (1.0 - k) + k);
	return GL * GV;
}

vec3 F_Schlick(float cosTheta, vec3 F0)
{
	return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

vec3 F_SchlickR(float cosTheta, vec3 F0, float roughness)
{
	return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(1.0 - cosTheta, 5.0);
}

vec3 prefilter_reflection(vec3 R, float roughness) {
    const float MAX_REFLECTION_LOD = 9.0;
    float lod = roughness * MAX_REFLECTION_LOD;
    float lodf = floor(lod);
    float lodc = ceil(lod);
    vec3 a = textureLod(prefilter_map, R, lodf).rgb;
    vec3 b = textureLod(prefilter_map, R, lodc).rgb;
    return mix(a, b, lod - lodf);
}

vec3 specular_contribution(vec3 L, vec3 V, vec3 N, vec3 F0, vec3 raw_color, float metallic, float roughness)
{
	// Precalculate vectors and dot products	
	vec3 H = normalize (V + L);
	float dotNH = clamp(dot(N, H), 0.0, 1.0);
	float dotNV = clamp(dot(N, V), 0.0, 1.0);
	float dotNL = clamp(dot(N, L), 0.0, 1.0);

	// Light color fixed
	vec3 lightColor = vec3(1.0);

	vec3 color = vec3(0.0);

	if (dotNL > 0.0) {
		// D = Normal distribution (Distribution of the microfacets)
		float D = D_GGX(dotNH, roughness); 
		// G = Geometric shadowing term (Microfacets shadowing)
		float G = G_SchlicksmithGGX(dotNL, dotNV, roughness);
		// F = Fresnel factor (Reflectance depending on angle of incidence)
		vec3 F = F_Schlick(dotNV, F0);		
		vec3 spec = D * F * G / (4.0 * dotNL * dotNV + 0.001);		
		vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);			
		color += (kD * raw_color / PI + spec) * dotNL;
	}

	return color;
}

void main() {
    vec3 N = get_normal();
    vec3 V = normalize(ubo.cam_pos.rgb - frag_uvw);
    vec3 R = reflect(-V, N);
    vec3 raw_color = get_color();

    vec2 metallic_roughness = get_metallic_roughness();
    float metallic = metallic_roughness.x;
    float roughness = metallic_roughness.y;

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, raw_color, metallic);

    vec3 light_pos = vec3(5.0, 5.0, 5.0);
    vec3 L = normalize(light_pos - frag_uvw);
    vec3 Lo = specular_contribution(L, V, N, F0, raw_color, metallic, roughness);

    vec2 brdf = texture(brdf_map, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 reflection = prefilter_reflection(R, roughness).rgb;
    vec3 irradiance = texture(irradiance_map, N).rgb;

    vec3 diffuse = irradiance * raw_color;

    vec3 F = F_SchlickR(max(dot(N, V), 0.0), F0, roughness);

    vec3 specular = reflection * (F * (brdf.x + brdf.y));

    vec3 kD = 1.0 - F;
    kD *= 1.0 - metallic;
    vec3 ambient = (kD * diffuse + specular);

    vec3 color = ambient + Lo;

    if ((pco.material_flag & OCCLUSION_TEXTURE_BIT ) > 0) {
        float ao = texture(ao_map, in_uv).r;
        color = mix(color, color * ao, OCCLUSION_STRENGTH);
    }

    vec3 emissive = get_emissive();
    color += emissive * EMISSIVE_STRENGTH;


    color = color / (color + vec3(1.0));

    color = pow(color, vec3(1.0 / 2.2));

    out_color = vec4(color, 1.0);

}