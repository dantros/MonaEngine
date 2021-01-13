#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 5) in vec4 aBoneIndices;
layout (location = 6) in vec4 aBoneWeights;
layout(location = 0) uniform mat4 mvpMatrix;
layout(location = 1) uniform mat4 modelMatrix;
layout(location = 2) uniform mat4 modelInverseTransposeMatrix;

layout(location = 10) uniform mat4 boneTransforms[${MAX_BONES}];


out vec3 worldPos;
out vec3 normal;

void main()
{
	//boneTransform representa la matriz al aplicar la piel a este vertice
	mat4 boneTransform  =  mat4(0.0);
	boneTransform  +=    boneTransforms[int(aBoneIndices.x)] * aBoneWeights.x;
	boneTransform  +=    boneTransforms[int(aBoneIndices.y)] * aBoneWeights.y;
	boneTransform  +=    boneTransforms[int(aBoneIndices.z)] * aBoneWeights.z;
	boneTransform  +=    boneTransforms[int(aBoneIndices.w)] * aBoneWeights.w;
	mat4 finalModelTransform = modelMatrix * boneTransform;
	worldPos = vec3( finalModelTransform * vec4(aPos, 1.0f));
	normal = normalize(mat3(transpose(inverse(finalModelTransform))) * aNormal);
	gl_Position = mvpMatrix * boneTransform * vec4(aPos,1.0);

}