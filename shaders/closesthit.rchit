#version 460
#extension GL_EXT_ray_tracing : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int64 : require
#extension GL_EXT_buffer_reference2 : require
#extension GL_ARB_gpu_shader_int64 : require
#extension GL_EXT_nonuniform_qualifier : require


#define PI 3.1415926535897932384626433832795

struct Material {
    vec4 base_color;
    vec4 metallic_roughness;
    uint64_t albedo_texture_idx;
    uint64_t  normal_texture_idx;
	uint64_t  occlusion_texture_idx;
	uint64_t  emissive_texture_idx;
	uint64_t  metallic_roughness_texture_idx;
};

struct SubmeshAccessInfo {
    uint64_t vert_buf_addr;
    uint64_t idx_buf_addr;
    uint64_t material_idx;
};

struct Vertex {
	vec3 pos;
	vec3 norm;
	vec2 uv;
	vec4 joint;
	vec4 weight;
	vec4 color;
};


hitAttributeEXT vec2 attribs;

layout(location = 0) rayPayloadInEXT vec3 hit_val;

layout(buffer_reference, scalar) buffer Vertices {
    Vertex v[];
};

layout(buffer_reference, scalar) buffer Indices {
    ivec3 i[]; 
};

layout(set = 0, binding = 2) buffer MaterialArray {
    Material m[];
} material_array;

layout(set = 0, binding = 3) buffer SubmeshAccessInfoArray {
    SubmeshAccessInfo m[];
} submesh_access_array;

layout(set = 0, binding = 4) uniform texture2D textures[];

layout(set = 0, binding = 5) uniform sampler samp;

layout(set = 0, binding = 6) uniform samplerCube irradiance_map;
layout(set = 0, binding = 7) uniform samplerCube prefilter_map;
layout(set = 0, binding = 8) uniform sampler2D brdf_map;



layout(push_constant) uniform PCO {
    vec4 clear_color;   
    vec4 light_position;
    float light_intensity;
} pco;

const float OCCLUSION_STRENGTH = 0.5f;
const float EMISSIVE_STRENGTH = 1.0f;



vec3 get_normal(Vertex v0, Vertex v1, Vertex v2, vec3 tangent_normal, vec3 in_normal)
{
    vec3 v0_world_pos = vec3(gl_ObjectToWorldEXT * vec4(v0.pos, 1.0));
    vec3 v1_world_pos = vec3(gl_ObjectToWorldEXT * vec4(v1.pos, 1.0));
    vec3 v2_world_pos = vec3(gl_ObjectToWorldEXT * vec4(v2.pos, 1.0));

    
    vec3 Q1  = v1_world_pos - v0_world_pos;
    vec3 Q2  = v2_world_pos - v0_world_pos;
    vec2 st1 = v1.uv - v0.uv;
    vec2 st2 = v2.uv - v0.uv;

    vec3 N   = normalize(in_normal);
    vec3 T  = normalize(Q1*st2.t - Q2*st1.t);
    vec3 B  = -normalize(cross(N, T));
    mat3 TBN = mat3(T, B, N);

    return normalize(TBN * tangent_normal);
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

vec4 read_texture(int texture_idx, vec2 uv) {
    return texture(sampler2D(textures[texture_idx], samp), uv);
}

void  main() {
    SubmeshAccessInfo submesh_ainfo = submesh_access_array.m[gl_InstanceCustomIndexEXT];
    Vertices vertices = Vertices(submesh_ainfo.vert_buf_addr);
    Indices indices = Indices(submesh_ainfo.idx_buf_addr);
    Material material = material_array.m[int(submesh_ainfo.material_idx)];
    
    ivec3 ind = indices.i[gl_PrimitiveID];

    Vertex v0 = vertices.v[ind.x];
    Vertex v1 = vertices.v[ind.y];
    Vertex v2 = vertices.v[ind.z];

    vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    vec3 pos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    vec3 world_pos = vec3(gl_ObjectToWorldEXT * vec4(pos, 1.0));
    vec2 uv = v0.uv * barycentrics.x + v1.uv * barycentrics.y + v2.uv * barycentrics.z;
    vec3 in_normal = v0.norm * barycentrics.x + v1.norm * barycentrics.y + v2.norm * barycentrics.z;

    in_normal = normalize(vec3(in_normal * gl_WorldToObjectEXT));
    vec3 tangent_normal = texture(sampler2D(textures[int(material.normal_texture_idx)], samp), uv).xyz * 2.0 - 1.0;


    vec3 N = get_normal(v0, v1, v2, tangent_normal, in_normal);
    vec3 V = -normalize(gl_WorldRayDirectionEXT);
    vec3 R = reflect(-V, N);
    vec3 raw_color = read_texture(int(material.albedo_texture_idx), uv).xyz;
    vec2 metallic_roughness = read_texture(int(material.metallic_roughness_texture_idx), uv).bg;

    float metallic = metallic_roughness.x;
    float roughness = metallic_roughness.y;

    vec3 F0 = vec3(0.04);
    F0 = mix(F0, raw_color, metallic);

    vec3 light_pos = vec3(5.0, 5.0, 5.0);
    vec3 L = normalize(light_pos - world_pos);
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

    float ao = read_texture(int(material.occlusion_texture_idx), uv).r;
    color = mix(color, color * ao, OCCLUSION_STRENGTH);

    vec3 emissive = read_texture(int(material.emissive_texture_idx), uv).rgb;
    color += emissive * EMISSIVE_STRENGTH;

    color = color / (color + vec3(1.0));

    color = pow(color, vec3(1.0 / 2.2));

    hit_val = color;
}
