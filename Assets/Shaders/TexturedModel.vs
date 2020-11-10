#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout(location = 0) uniform mat4 projectionMatrix;
layout(location = 1) uniform mat4 viewMatrix;
layout(location = 2) uniform mat4 modelMatrix;

out vec3 normal;
out vec2 texCoord;

void main()
{
	normal = mat3(transpose(inverse(modelMatrix))) * aNormal;
	texCoord = aTexCoord;
	gl_Position = projectionMatrix * viewMatrix *modelMatrix * vec4(aPos,1.0);

}