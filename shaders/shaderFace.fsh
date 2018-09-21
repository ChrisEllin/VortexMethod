#version 330 core
#extension GL_ARB_separate_shader_objects: enable

in vec3 pos;
in vec3 col;
in vec3 norm;
in vec4 cameraPos;
in vec4 shadowPos;

out vec3 color;
uniform sampler2D shadowMap;

void main(){
//    float visibility = 1.0;
//    if (texture(shadowMap, shadowPos.xy).r < (shadowPos.z)/shadowPos.w)
//        visibility = 0.5;
    float visibility = texture(shadowMap, shadowPos.xy).r;

    vec3 vecToLight = vec3(0, -10, 0);
    vecToLight = vecToLight - pos;

    vec3 n = normalize(pos);
    vec3 l = normalize(-vecToLight);
    vec3 e = normalize(vec3(0, 0, 0) - cameraPos.xyz);
    vec3 r = reflect(-l, n);

    vec3 ambientColor = col * 0.2;

    float cosTheta = clamp( dot( n,l ), 0,1 );
    float cosAlpha = clamp( dot( e,r ), 0,1 );
    color =
//       vec3(visibility);
        /*visibility * */col * cosTheta
        + 0.5 * vec3(1, 1, 1) * pow(cosAlpha,5);
        + ambientColor;
//        texture(shadowMap, shadowPos.xy).rgb;
}
