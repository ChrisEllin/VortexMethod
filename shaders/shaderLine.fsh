#version 330 core
#extension GL_ARB_separate_shader_objects: enable

in vec3 col;

out vec3 color;

void main() {
   color = col;
}
