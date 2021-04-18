#version 450 core
layout (location = 0) in vec3 aPos;
layout(location = 0) uniform mat4 mvpMatrix;
layout(location = 1) uniform mat4 modelMatrix;
layout(location = 2) uniform mat4 modelInverseTransposeMatrix;


void main()
{
	gl_Position = mvpMatrix * vec4(aPos,1.0);
}