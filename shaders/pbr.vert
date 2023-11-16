#version 450

const int MAX_NUM_JOINTS = 256;

layout(binding = 0) uniform CameraUBO {
    mat4 proj_view;
} camera_ubo;

layout(binding = 1) uniform JointUBO {
    mat4 M[MAX_NUM_JOINTS];
    float is_skinned;
} joint_ubo;

layout(push_constant) uniform PCO {
    mat4 model;
} pco;

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec4 joint;
layout(location = 4) in vec4 weight;
layout(location = 5) in vec4 color; 

layout(location = 0) out vec3 out_normal;
layout(location = 1) out vec2 out_uv;
layout(location = 2) out vec3 frag_uvw;
layout(location = 5) out vec4 out_color;

void main() {
    if (joint_ubo.is_skinned > 0) {
    mat4 skin_M = weight.x * joint_ubo.M[int(joint.x)] + 
    weight.y * joint_ubo.M[int(joint.y)] + 
    weight.z * joint_ubo.M[int(joint.z)] + 
    weight.w * joint_ubo.M[int(joint.w)];
    gl_Position = camera_ubo.proj_view * pco.model * skin_M * vec4(position, 1.0);
    out_normal = normalize(transpose(inverse(mat3(pco.model * skin_M))) * normal);

    } else {
        gl_Position = camera_ubo.proj_view * pco.model * vec4(position, 1.0);
        out_normal = normalize(transpose(inverse(mat3(pco.model))) * normal);
    }
    frag_uvw = vec3(pco.model * vec4(position, 1.0));
    out_uv = uv;
    out_color = color;
}