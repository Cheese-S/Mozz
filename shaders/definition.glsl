const int ColorPayloadIndex = 0;
const int ShadowPayloadIndex = 1;

const uint MAX_DEPTH = 4;

struct ColorPayload {
    vec3 color;
    uint depth;
    float curr_ior; 
};

