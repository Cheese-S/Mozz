#version 460
#extension GL_EXT_ray_tracing : enable

#include "definition.glsl"

layout(location = 0) rayPayloadInEXT ColorPayload color_payload;

layout(set = 0, binding = 7) uniform samplerCube background;


layout(push_constant) uniform PCO {
    vec4 clear_color;   
    vec4 light_position;
    float light_intensity;
} pco;

void main() {
    color_payload.color = texture(background, gl_WorldRayDirectionEXT).rgb;
    // color_payload.color = pco.clear_color.rgb;
}