#version 450 core
layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

out vec4 Normal;

void main(){
	Normal = view * model * vec4(normal, 0);
	gl_Position = projection * view * model * vec4(position, 1.0);
}