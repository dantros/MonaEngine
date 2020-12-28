#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
//layout (location = 2) in vec2 aTexCoord;
//layout (location = 3) in vec3 aTangent;
//layout (location = 4) in vec3 aBitangent;
layout (location = 5) in vec4 aBoneIndices;
layout (location = 6) in vec4 aBoneWeights;
layout(location = 0) uniform mat4 mvpMatrix;
layout(location = 1) uniform mat4 modelMatrix;
layout(location = 2) uniform mat4 modelInverseTransposeMatrix;

layout(location = 10) uniform mat4 boneTransforms[${MAX_BONES}];

out vec3 normal;
out vec3 worldPos;

void main()
{
	mat4 boneTransform  =  mat4(0.0);
	boneTransform  +=    boneTransforms[int(aBoneIndices.x)] * aBoneWeights.x;
	boneTransform  +=    boneTransforms[int(aBoneIndices.y)] * aBoneWeights.y;
	boneTransform  +=    boneTransforms[int(aBoneIndices.z)] * aBoneWeights.z;
	boneTransform  +=    boneTransforms[int(aBoneIndices.w)] * aBoneWeights.w;	
	worldPos = vec3(modelMatrix * boneTransform * vec4(aPos, 1.0f));
	normal = mat3(transpose(inverse(modelMatrix * boneTransform))) * aNormal;
	gl_Position = mvpMatrix * boneTransform * vec4(aPos,1.0);

}