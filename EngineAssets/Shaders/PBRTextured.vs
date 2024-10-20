#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;
layout (location = 3) in vec3 aTangent;
layout (location = 4) in vec3 aBitangent;
layout(location = 0) uniform mat4 mvpMatrix;
layout(location = 1) uniform mat4 modelMatrix;
layout(location = 2) uniform mat4 modelInverseTransposeMatrix;

//out vec3 normal;
out vec3 worldPos;
out vec2 texCoord;
out vec3 normal;
out vec3 tangent;
out vec3 bitangent;

void main()
{
	normal = normalize(mat3(modelInverseTransposeMatrix) * aNormal);
	tangent = normalize(mat3(modelMatrix)* aTangent);
	bitangent = normalize(mat3(modelMatrix)* aBitangent);

	texCoord = aTexCoord;
	worldPos = vec3(modelMatrix * vec4(aPos,1.0f));
	gl_Position = mvpMatrix * vec4(aPos,1.0f);

}