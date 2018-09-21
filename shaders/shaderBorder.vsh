#version 330 core
#extension GL_ARB_separate_shader_objects: enable

layout(location = 0) in vec4 posAttr;

uniform mat4 matrixView;

void main() {
   gl_Position = matrixView * posAttr;
}

