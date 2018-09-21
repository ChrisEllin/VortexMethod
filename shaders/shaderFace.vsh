#version 330 core
#extension GL_ARB_separate_shader_objects: enable

layout(location = 0) in vec4 posAttr;
layout(location = 1) in vec3 colAttr;
layout(location = 2) in vec3 normAttr;

out vec3 pos;
out vec3 col;
out vec3 norm;
out vec4 shadowPos;
out vec4 cameraPos;

uniform mat4 matrixView;
//uniform sampler2DShadow shadowMap;
uniform mat4 matrixDepth;

void main(){
    gl_Position =  matrixView * posAttr;
    shadowPos = matrixDepth * posAttr;
    norm = (matrixView * vec4(normAttr, 1.0)).xyz;
    pos = (matrixView*posAttr).xyz;
    cameraPos = posAttr;
    col = colAttr;
}
