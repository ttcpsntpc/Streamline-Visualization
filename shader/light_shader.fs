#version 330 core
out vec4 FragColor;

in vec3 ourColor;
in vec3 TexCoord;
in vec3 Normal;
in vec3 FragPos;

uniform sampler1D texture0;
uniform bool hasTF;

// function prototypes
vec3 PhongShading(vec3 gradient,vec3 color);

void main(){
    if(hasTF == true)    
        FragColor = vec4(texture(texture0, TexCoord.s));
    else
        FragColor = vec4(ourColor, 1.0f);
}
