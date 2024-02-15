#version 460
#extension GL_EXT_ray_tracing : enable
#extension GL_EXT_scalar_block_layout : enable
#extension GL_EXT_shader_explicit_arithmetic_types_int64 : require
#extension GL_EXT_buffer_reference2 : require
#extension GL_ARB_gpu_shader_int64 : require
#extension GL_EXT_nonuniform_qualifier : require
#extension GL_GOOGLE_include_directive : enable

#include "definition.glsl"


#define PI 3.1415926535897932384626433832795

const uint UNDEFINED_TEXTURE = 1024;
const vec3 LIGHT_POS = vec3(0.5, 0.5, 0.5);

struct Material {
    vec4 base_color;
    vec4 metallic_roughness;
    uint64_t albedo_texture_idx;
    uint64_t  normal_texture_idx;
	uint64_t  occlusion_texture_idx;
	uint64_t  emissive_texture_idx;
	uint64_t  metallic_roughness_texture_idx;
    float ior;
    float pad;
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

layout(location = ColorPayloadIndex) rayPayloadInEXT ColorPayload color_payload;

layout(location = ShadowPayloadIndex) rayPayloadEXT bool is_shadowed;



layout(buffer_reference, scalar) buffer Vertices {
    Vertex v[];
};

layout(buffer_reference, scalar) buffer Indices {
    ivec3 i[]; 
};

layout(set = 0, binding = 0) uniform accelerationStructureEXT tlas; 

layout(set = 0, binding = 2, scalar) buffer MaterialArray {
    Material m[];
} material_array;

layout(set = 0, binding = 3, scalar) buffer SubmeshAccessInfoArray {
    SubmeshAccessInfo m[];
} submesh_access_array;

layout(set = 0, binding = 4) uniform samplerCube irradiance_map;
layout(set = 0, binding = 5) uniform samplerCube prefilter_map;
layout(set = 0, binding = 6) uniform sampler2D brdf_map;
layout(set = 0, binding = 8) uniform sampler2D textures[];



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

float fresnel(vec3 D, vec3 N, float eta_incident, float eta_trans) {
    float cos_incident = dot(D, N);
    float sin_trans = eta_incident / eta_trans * sqrt(max(0, 1 - cos_incident * cos_incident));
    if (sin_trans >= 1.0f) {
        return 1.0f; 
    } else {
        float cos_trans = sqrt(max(0, 1 - sin_trans * sin_trans));
        cos_incident = abs(cos_incident);   
        float rS = ((eta_trans * cos_incident) - (eta_incident * cos_trans)) /
                   ((eta_trans * cos_incident) + (eta_incident * cos_trans));
        float rP = ((eta_incident * cos_incident) - (eta_trans * cos_trans)) /
                   ((eta_incident * cos_incident) + (eta_trans * cos_trans));
        return (rS * rS + rP * rP) / 2.0f;
    }
    
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
    return texture(textures[texture_idx], uv);
}

vec3 get_albedo(Material mat, vec2 uv) {
    if (mat.albedo_texture_idx != UNDEFINED_TEXTURE) {
        return read_texture(int(mat.albedo_texture_idx), uv).xyz;
    }
    return mat.base_color.xyz;
}

vec2 get_metallic_roughness(Material mat, vec2 uv) {
    if (mat.metallic_roughness_texture_idx != UNDEFINED_TEXTURE) {
        return read_texture(int(mat.metallic_roughness_texture_idx), uv).bg;
    }
    return mat.metallic_roughness.bg;
}

float get_ao(Material mat, vec2 uv) {
    if (mat.occlusion_texture_idx != UNDEFINED_TEXTURE) {
        return read_texture(int(mat.occlusion_texture_idx), uv).r;
    }
    return 1.0f;
}

vec3 get_emissive(Material mat, vec2 uv) {
    if (mat.emissive_texture_idx != UNDEFINED_TEXTURE) {
        return read_texture(int(mat.emissive_texture_idx), uv).rgb;
    }
    return vec3(0, 0, 0);
}

bool is_opaque(float ior) {
    return ior < 0.0f;
}

float trace_shadow_ray(vec3 origin, vec3 dir, float light_distance) {
    float t_min = 0.0001;
    float t_max = light_distance;
    uint flags = gl_RayFlagsOpaqueEXT | gl_RayFlagsTerminateOnFirstHitEXT | gl_RayFlagsSkipClosestHitShaderEXT;
    is_shadowed = true;
    traceRayEXT(
        tlas,
        flags,
        0xFF,
        0,
        0,
        1,
        origin,
        t_min,
        dir,
        t_max,
        ShadowPayloadIndex
    );

    if (is_shadowed) {
        return 0.3;
    } else {
        return 1;
    }
}

vec3 trace_reflection_ray(vec3 origin, vec3 dir) {
    float t_min = 0.001;
    float t_max = 1000;
    uint flags = gl_RayFlagsOpaqueEXT;
    color_payload.depth++;
    traceRayEXT(
        tlas,
        flags,
        0xFF,
        0,
        0,
        0,
        origin,
        t_min, 
        dir, 
        t_max,
        ColorPayloadIndex
    );
    color_payload.depth--;
    return color_payload.color;
}

vec3 trace_refraction_ray(vec3 N, vec3 D, vec3 origin, float ior) {
    float N_dot_D = dot(D, N);
    vec3 refract_N;
    if (N_dot_D > 0.0f) {
        refract_N = -N;
    } else {
        refract_N = N;
    }
    vec3 dir = refract(D, refract_N, color_payload.curr_ior / ior);
    float t_min = 0.001;
    float t_max = 1000;
    float old_ior = color_payload.curr_ior;
    uint flags = gl_RayFlagsOpaqueEXT;
    color_payload.depth++;
    color_payload.curr_ior = ior;
    traceRayEXT(
        tlas,
        flags,
        0xFF,
        0,
        0,
        0,
        origin,
        t_min,
        dir,
        t_max, 
        ColorPayloadIndex
    );
    color_payload.depth--;
    color_payload.curr_ior = old_ior;
    return color_payload.color;
}



void  main() {
    /* ------------------------ RETURN IF HIT DEPTH LIMIT ----------------------- */
    if (color_payload.depth > MAX_DEPTH) {
        color_payload.color = vec3(0, 0, 0);
        return;
    }
    /* -------------------------------- UNPACKING ------------------------------- */
    SubmeshAccessInfo submesh_ainfo = submesh_access_array.m[gl_InstanceCustomIndexEXT];
    Vertices vertices = Vertices(submesh_ainfo.vert_buf_addr);
    Indices indices = Indices(submesh_ainfo.idx_buf_addr);
    Material mat = material_array.m[int(submesh_ainfo.material_idx)];
    vec3 barycentrics = vec3(1.0 - attribs.x - attribs.y, attribs.x, attribs.y);

    ivec3 ind = indices.i[gl_PrimitiveID];
    Vertex v0 = vertices.v[ind.x];
    Vertex v1 = vertices.v[ind.y];
    Vertex v2 = vertices.v[ind.z];

    vec3 pos = v0.pos * barycentrics.x + v1.pos * barycentrics.y + v2.pos * barycentrics.z;
    vec3 world_pos = vec3(gl_ObjectToWorldEXT * vec4(pos, 1.0));
    vec2 uv = v0.uv * barycentrics.x + v1.uv * barycentrics.y + v2.uv * barycentrics.z;
    vec3 vertex_normal = v0.norm * barycentrics.x + v1.norm * barycentrics.y + v2.norm * barycentrics.z;
    vec3 world_normal = normalize(vec3(vertex_normal * gl_WorldToObjectEXT));
    vec3 N;
    if (mat.normal_texture_idx == UNDEFINED_TEXTURE) {
        N = world_normal;
    } else {
        vec3 tangent_normal = read_texture(int(mat.normal_texture_idx), uv).xyz * 2.0 - 1.0;
        N = get_normal(v0, v1, v2, tangent_normal, world_normal);
    }

    /* ------------------------------- SCENE INFO ------------------------------- */
    vec3 origin = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * gl_HitTEXT;
    vec3 F0 = vec3(0.04);
    vec3 D = normalize(gl_WorldRayDirectionEXT);
    vec3 V = -D;
    vec3 L = normalize(LIGHT_POS - world_pos);
    vec3 R = reflect(D, N);


    vec3 albedo = get_albedo(mat, uv);
    vec2 metallic_roughness = get_metallic_roughness(mat, uv);
    float metallic = metallic_roughness.x;    
    float roughness = metallic_roughness.y;


    /* ------------------------------- DIRECT SHADING -------------------- */
    vec2 brdf = texture(brdf_map, vec2(max(dot(N, V), 0.0), roughness)).rg;
    vec3 ibl_reflection = prefilter_reflection(R, roughness).rgb;
    vec3 irradiance = texture(irradiance_map, N).rgb;
    vec3 diffuse = irradiance * albedo;
    vec3 F = F_SchlickR(max(dot(N, V), 0.0), F0, roughness);
    // vec3 specular = ibl_reflection * (F * (brdf.x + brdf.y));
    vec3 kD = 1.0 - F;
    kD *= 1.0 - metallic;

    vec3 color = kD * diffuse;

    vec3 Lo = vec3(0, 0, 0);
    if (is_opaque(mat.ior)) {
        float dot_NL = clamp(dot(N, L), 0.0, 1.0);
        if (dot_NL > 0.000001) {
            // light info
            vec3 H = normalize(V + L); 
            float light_distance = length(LIGHT_POS -  world_pos);
            float attenuation = trace_shadow_ray(origin, L, light_distance);
            vec3 radiance = attenuation * vec3(4.0f, 4.0f, 4.0f);

            float dot_NV = clamp(dot(N, V), 0.0, 1.0);
            float dot_NH = clamp(dot(N, H), 0.0, 1.0);

            // BRDF
            float D = D_GGX(dot_NH, roughness);
            float G = G_SchlicksmithGGX(dot_NL, dot_NV, roughness);

            vec3 numerator = D * F * G;
            float denominator = 4.0f * dot_NV * dot_NL + 0.0001;
            vec3 specular = numerator / denominator;
            Lo += (kD * albedo / PI + specular) * radiance * dot_NL;
            color += kD * diffuse;
        }
    }
    color += Lo; 
    /* ----------------------------- SECONDARY RAYS ----------------------------- */

    vec3 reflection = vec3(0, 0, 0);
    vec3 refraction = vec3(0, 0, 0);
    if (metallic > 0.0000001f) {
        reflection = trace_reflection_ray(origin, R);
        reflection *= (1 - roughness) * (1 - roughness) * metallic * albedo;
        
        color += reflection;
    }

    if(mat.ior > 0.0) {
        float F = fresnel(D, N, color_payload.curr_ior, mat.ior);
        refraction = trace_refraction_ray(N, D, origin, mat.ior);
        if (dot(D, N) < 0.0f && F > 0.0f) {
            reflection = trace_reflection_ray(origin, R);
        }

        if (F < 0.99999999f) {
            refraction = trace_refraction_ray(N, D, origin, mat.ior);
        }
        color += mix(refraction, reflection, F);
    }

    // vec3 emissive = get_emissive(mat, uv);
    // color += emissive;

    
    color_payload.color = color;


}
