#version 330 core
in vec4 posAttr;
//out vec4 pos;
uniform mat4 matrixView;
void main(){
    gl_Position =  matrixView * posAttr;
//    pos = matrixView * posAttr;
}
