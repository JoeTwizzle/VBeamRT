#version 460
layout(location = 0) in vec2 texCoord;
layout(set = 0, binding = 0) uniform sampler _MainSampler;
layout(set = 0, binding = 1) uniform texture2D _MainTexture;
layout(location = 0) out vec4 color;

vec3 Tonemap_ACES(vec3 x)
{
    // Narkowicz 2015, "ACES Filmic Tone Mapping Curve"
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return (x * (a * x + b)) / (x * (c * x + d) + e);
}
// https://www.shadertoy.com/view/WltSRB
// https://twitter.com/jimhejl/status/1137559578030354437
vec3 ToneMapFilmicALU(vec3 x)
{
    x *= 0.665;

    x = max(vec3(0.0), x);
    x = (x * (6.2 * x + 0.5)) / (x * (6.2 * x + 1.7) + 0.06);
    x = pow(x, vec3(2.2));// using gamma instead of sRGB_EOTF + without x - 0.004f looks about the same

    return x;
}

const mat3 ACESOutputMat = mat3
(
        1.60475, -0.53108, -0.07367,
    -0.10208,  1.10813, -0.00605,
    -0.00327, -0.07276,  1.07602
);

const mat3 RRT_SAT = mat3
(
    0.970889, 0.026963, 0.002148,
    0.010889, 0.986963, 0.002148,
    0.010889, 0.026963, 0.962148
);

vec3 ToneTF2(vec3 x)
{
    vec3 a = (x            + 0.0822192) * x;
    vec3 b = (x * 0.983521 + 0.5001330) * x + 0.274064;

    return a / b;
}

vec3 Tonemap_ACESFitted2(vec3 acescg)
{
    vec3 color = acescg * RRT_SAT;

    color = ToneTF2(color); 

    color = color * ACESOutputMat;
    //color = ToneMapFilmicALU(color);

    return color;
}

void main()
{
    vec3 rawColor = texture(sampler2D(_MainTexture, _MainSampler), texCoord).rgb;
    color = vec4(Tonemap_ACES(rawColor), 1.0);
    //color = vec4(Tonemap_ACES(rawColor.x),Tonemap_ACES(rawColor.y),Tonemap_ACES(rawColor.z), 1.0);
//    color = vec4(rawColor, 1.0);
}