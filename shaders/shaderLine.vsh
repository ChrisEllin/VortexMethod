#version 330 core
#extension GL_ARB_separate_shader_objects: enable

/*layout(location = 0) */in vec4 posAttr;
/*layout(location = 1) */uniform vec3 colAttr;

out vec3 col;

uniform mat4 matrixView;

void main() {
   col = colAttr;
   gl_Position = matrixView * posAttr;
}
