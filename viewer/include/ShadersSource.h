#pragma once

#include <string>
using namespace std;

const string flatVert = R"(
#version 410 core
layout (location = 0) in vec3 inPosition;
layout (location = 2) in vec3 inColor;

out vec3 color;

layout (std140) uniform Transform {
    mat4 projection;
    mat4 view;
    mat4 view3D;
    mat4 model;
};

void main()
{
    gl_Position = projection * view * model * vec4(inPosition, 1.0);
    color = inColor;
}
)";

const string flatFrag = R"(
#version 410 core
in vec3 color;

out vec4 fragColor;

void main()
{
    fragColor = vec4(color, 1.0);
}
)";

const string wireframeVert = R"(
#version 410 core
layout (location = 0) in vec3 inPosition;
layout (location = 1) in vec3 inBarycenter;

out vec3 barycenter;

layout (std140) uniform Transform {
    mat4 projection;
    mat4 view;
    mat4 view3D;
    mat4 model;
};

void main()
{
    gl_Position = projection * view * model * vec4(inPosition, 1.0);
    barycenter = inBarycenter;
}
)";

const string wireframeFrag = R"(
#version 410 core
in vec3 barycenter;

out vec4 fragColor;

float edgeFactor()
{
    vec3 d = fwidth(barycenter);
    vec3 a3 = smoothstep(vec3(0.0), d, barycenter);
    return min(min(a3.x, a3.y), a3.z);
}

void main()
{
    fragColor = vec4(0.0, 0.0, 0.0, (1.0 - edgeFactor())*0.4);
}
)";

const string modelVert = R"(
#version 410 core
layout (location = 0) in vec3 inPosition;
layout (location = 2) in vec3 inColor;
layout (location = 3) in vec3 inNormal;
layout (location = 4) in vec3 inTexCoords;

out vec3 position;
out vec3 texCoords;
out vec3 normal;
out vec3 color;

layout (std140) uniform Transform {
    mat4 projection;
    mat4 view;
    mat4 view3D;
    mat4 model;
};

void main()
{
    gl_Position = projection * view * model * vec4(inPosition, 1.0);
    position = vec3(gl_Position);
    texCoords = inTexCoords;
    normal = -normalize(vec3(model * view3D * vec4(inNormal, 0.0)));
    color = inColor;
}
)";

const string modelFrag = R"(
#version 410 core
in vec3 position;
in vec3 texCoords;
in vec3 normal;
in vec3 color;

out vec4 fragColor;

layout (std140) uniform Light {
    vec3 position;
    vec3 color;
} light;

uniform vec3 eye;
uniform float patternFactor;
uniform float patternSize;
uniform int drawUVSphere;
uniform int pattern;

void main()
{
    vec3 L = normalize(light.position - position);
    vec3 E = normalize(eye - position);
    vec3 H = normalize(L + E);
    vec3 N = normal;
    vec3 rgb = color;
    
    if (pattern == 3) {
        // circle
        mat3[8] U;
        U[0] = mat3(0.891157, -0.307877, 0.333243, 0.367113, 0.920917, -0.130915, -0.266584, 0.239004, 0.933708);
        U[1] = mat3(-0.178366, -0.983941, -0.00681949, -0.895065, 0.165126, -0.414237, 0.408711, -0.067782, -0.910144);
        U[2] = mat3(-0.340809, -0.660468, -0.669053, 0.658574, 0.340147, -0.671253, 0.670917, -0.66939, 0.319041);
        U[3] = mat3(0.977242, -0.1618, 0.137182, -0.205916, -0.568204, 0.796707, -0.05096, -0.806824, -0.58859);
        U[4] = mat3(-0.174663, 0.7001, -0.692353, 0.934645, -0.103303, -0.340246, -0.309728, -0.706532, -0.636302);
        U[5] = mat3(-0.426529, -0.84799, -0.314619, -0.857865, 0.489509, -0.156362, 0.286602, 0.203208, -0.936251);
        U[6] = mat3(-0.776165, 0.325758, -0.539861, -0.551067, 0.065628, 0.831876, 0.30642, 0.943173, 0.128576);
        U[7] = mat3(-0.372001, 0.662937, -0.649715, 0.570619, 0.715396, 0.403241, 0.732127, -0.220733, -0.644413);
        
        vec3 colors[8];
        colors[0] = vec3(1.0, 0.5, 0.0);
        colors[1] = vec3(0.0, 0.0, 1.0);
        colors[2] = vec3(1.0, 0.0, 0.0);
        colors[3] = vec3(0.7, 0.0, 0.7);
        colors[4] = vec3(0.2, 0.5, 0.9);
        colors[5] = vec3(0.2, 0.7, 0.3);
        colors[6] = vec3(0.9, 0.8, 0.1);
        colors[7] = vec3(0.1, 0.7, 0.7);

        for (int i = 0; i < 8; i++) {
            vec3 p = (0.2*patternSize + 0.35*i)*U[i]*texCoords;
            float d = length(p - (floor(p) + vec3(0.5, 0.5, 0.5)));
            float a = d < 0.3 ? (1 - patternFactor) : 1;
            
            rgb = a*rgb + (1 - a)*colors[i];
        }
        
    } else {
        float u = drawUVSphere == 1 ? atan(texCoords.z, texCoords.x)*0.5 : texCoords.x;
        float v = drawUVSphere == 1 ? asin(texCoords.y) : texCoords.y;
        float strength = 1.0;
        if (pattern == 2) {
            // checkerboard
            float fu = floor(u*patternSize);
            float fv = floor(v*patternSize);
            strength = patternFactor*mod(fu + fv, 2.0) + (1.0 - patternFactor);
            
        } else if( pattern == 1 ) {
            // grid 
            float fu = mod(u*patternSize, 1.0);
            float fv = mod(v*patternSize, 1.0);
            float a = 1;
            if (fu < 0.05 || fv < 0.05 || fu > 0.95 || fv > 0.95) a = 0;
            strength = patternFactor*a + (1.0 - patternFactor);
        
        } else {
           // none
           strength = 1.0;
        }
        
        rgb = strength*color;
    }
    
    if (!gl_FrontFacing) {
        N = -N;
        rgb.r *= 0.4;
        rgb.g *= 0.4;
        rgb.b *= 0.45;
    }

    float diffuse = abs(dot(N, L));
    float specular = pow(max(abs(dot(N, H)), 0.0), 12.0);
    float NE = max(abs(dot(N, E)), 0.0);
    float fresnel = pow(sqrt(1.0 - NE*NE), 12.0);
    
    fragColor = vec4((0.5*diffuse + 0.4*specular + 0.6*fresnel + 0.1)*rgb, 1.0);
}
)";
