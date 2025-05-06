#version 460
layout(location = 0) out vec2 texCoord;

void main()
{
    texCoord = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    vec2 xy = fma(texCoord, vec2(2.0) , vec2(-1.0));
    gl_Position = vec4(xy.x, -xy.y, 0.0, 1.0);
}