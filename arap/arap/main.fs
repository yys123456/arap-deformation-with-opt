#version 450 core
layout( location = 0 ) out vec4 color;

in vec4 Normal;
uniform sampler2D matcap_sampler;

vec3 matcap(){
	vec3 N = normalize(Normal.xyz);
	vec2 muv = N.xy * 0.5 + vec2(0.5,0.5);
	return texture(matcap_sampler, vec2(muv.x, 1.0 - muv.y)).rgb;
}

void main(){
	color = vec4(matcap(), 1.0);
}