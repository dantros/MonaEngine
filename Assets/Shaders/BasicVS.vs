#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout(location = 3) uniform mat4 viewMatrix;
layout(location = 4) uniform mat4 projectionMatrix;
layout(location = 5) uniform mat4 modelMatrix;

out vec3 normal;

void main()
{
	normal = mat3(transpose(inverse(modelMatrix))) * aNormal;
	gl_Position = projectionMatrix * viewMatrix *modelMatrix * vec4(aPos,1.0);

}