#include <algorithm>
#include <math.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "Animation/JointPose.hpp"
#include <iostream>
#include "Core/AssimpTransformations.hpp"
#include "Utilities/FuncUtils.hpp"
#include "Core/RootDirectory.hpp"



int main() {
	using jointIndex = uint32_t;
	struct AnimationTrack {
		std::vector<glm::vec3> positions;
		std::vector<glm::fquat> rotations;
		std::vector<glm::vec3> scales;
		std::vector<float> positionTimeStamps;
		std::vector<float> rotationTimeStamps;
		std::vector<float> scaleTimeStamps;
	};
	std::vector<AnimationTrack> m_animationTracks;
	std::vector<std::string> m_trackJointNames;
	std::vector<jointIndex> m_trackJointIndices;
	float m_duration = 1.0f;
	std::string filePath = Mona::SourcePath("Assets/Animations/female/running.fbx").string();
	Assimp::Importer importer;
	unsigned int postProcessFlags = aiProcess_Triangulate;
	const aiScene* scene = importer.ReadFile(filePath, postProcessFlags);
	if (!scene || scene->mNumAnimations == 0)
	{
		std::cout << "AnimationClip Error: Failed to open file with path " << filePath;
		return 0;
	}
	//Solo cargamos la primera animación
	aiAnimation* animation = scene->mAnimations[0];
	// Dado que los tiempos de muestreo de assimp estan generalmente en ticks se debe dividir casi todos 
	// los tiempos de sus objetos por tickspersecond. Por otro lado, assimp sigue la convencion de que si mTicksPerSecond 
	// es 0.0 entonces los timeStamps de las muestras de las animaciones estan en segundos y no ticks.
	float ticksPerSecond = animation->mTicksPerSecond != 0.0f ? animation->mTicksPerSecond : 1.0f;
	m_duration = animation->mDuration / animation->mTicksPerSecond;
	m_animationTracks.resize(animation->mNumChannels);
	for (uint32_t i = 0; i < animation->mNumChannels; i++) {
		aiNodeAnim* track = animation->mChannels[i];
		m_trackJointNames.push_back(track->mNodeName.C_Str());
		AnimationTrack& animationTrack = m_animationTracks[i];
		animationTrack.positions.reserve(track->mNumPositionKeys);
		animationTrack.positionTimeStamps.reserve(track->mNumPositionKeys);
		animationTrack.rotations.reserve(track->mNumRotationKeys);
		animationTrack.rotationTimeStamps.reserve(track->mNumRotationKeys);
		animationTrack.scales.reserve(track->mNumScalingKeys);
		animationTrack.scaleTimeStamps.reserve(track->mNumScalingKeys);
		for (uint32_t j = 0; j < track->mNumPositionKeys; j++) {
			animationTrack.positionTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
			animationTrack.positions.push_back(Mona::AssimpToGlmVec3(track->mPositionKeys[j].mValue));
		}
		for (uint32_t j = 0; j < track->mNumRotationKeys; j++) {
			animationTrack.rotationTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
			animationTrack.rotations.push_back(Mona::AssimpToGlmQuat(track->mRotationKeys[j].mValue));

		}
		for (uint32_t j = 0; j < track->mNumScalingKeys; j++) {
			animationTrack.scaleTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
			animationTrack.scales.push_back(Mona::AssimpToGlmVec3(track->mScalingKeys[j].mValue));

		}
	}
	std::cout << "keyframes: " << m_animationTracks[1].scaleTimeStamps.size();
	std::cout << Mona::funcUtils::vec3vecToString<glm::vec3>(m_animationTracks[1].scales);

	while (true) {

	}

	return 0;
}
