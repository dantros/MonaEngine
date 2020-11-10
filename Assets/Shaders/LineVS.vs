#version 450 core
layout(location = 0) uniform mat4 projectionMatrix;
layout(location = 1) uniform mat4 viewMatrix;
layout(location = 2) uniform vec3 fromPosition;
layout(location = 3) uniform vec3 toPosition;



void main()
{
	int a = (gl_VertexID%2);
	vec3 position = a * toPosition + ( 1 - a) * fromPosition;
	gl_Position = projectionMatrix * viewMatrix * vec4(position,1.0);

}