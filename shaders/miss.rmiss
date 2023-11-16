#version 460
#extension GL_EXT_ray_tracing : enable

layout(location = 0) rayPayloadInEXT vec3 hit_val;


layout(push_constant) uniform PCO {
    vec4 clear_color;   
    vec4 light_position;
    float light_intensity;
} pco;

void main() {
    hit_val = pco.clear_color.xyz;
}