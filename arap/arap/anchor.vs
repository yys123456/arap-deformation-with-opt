#version 450 core
layout (location = 0) in vec4 position_color;

uniform mat4 model;
uniform mat4 projection;
uniform mat4 view;

flat out int color;

void main(){
	gl_Position = projection * view * model * vec4(position_color.xyz, 1); // - 0.01 * camdir
	color = int(position_color.w);
}