#version 330 core
//in vec4 pos;
out vec4 color;
//out float fragmentdepth;


void main(){
    color = vec4(gl_FragCoord.z*gl_FragCoord.w);
//    fragmentdepth = gl_FragCoord.z*gl_FragCoord.w;
}
