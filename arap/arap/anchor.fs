#version 450 core
layout( location = 0 ) out vec4 Color;

flat in int color;

vec3 color_map[16] = vec3[](
	vec3(0,0,0),
	vec3(255,255,255),
	vec3(255,0,0),
	vec3(0,255,0),
	vec3(0,0,255),
	vec3(255,255,0),
	vec3(0,255,255),
	vec3(255,0,255),
	vec3(192,192,192),
	vec3(128,128,128),
	vec3(128,0,0),
	vec3(128,128,0),
	vec3(0,128,0),
	vec3(128,0,128),
	vec3(0,128,128),
	vec3(0,0,128)
);

void main(){
	Color = vec4(color_map[color % 16] / 255.0, 1.0);
}