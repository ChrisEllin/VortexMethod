#version 330 core

in vec3 UV;
out vec4 color;
uniform sampler2DShadow myTextureSampler;

void main(){
    color = vec4(vec3(texture( myTextureSampler, UV)), 1.0);
}
