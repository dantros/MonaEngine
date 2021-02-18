#include "AnimationClip.hpp"
#include <algorithm>
#include <math.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "../Core/AssimpTransformations.hpp"
#include "../Core/Log.hpp"
#include "Skeleton.hpp"
namespace Mona {
	AnimationClip::AnimationClip(const std::string& filePath,
		std::shared_ptr<Skeleton> skeleton,
		bool removeRootMotion)
	{
		MONA_ASSERT(skeleton != nullptr, "AnimationClip Error: Skeleton cannot be null");

		Assimp::Importer importer;
		unsigned int postProcessFlags = aiProcess_Triangulate;
		const aiScene* scene = importer.ReadFile(filePath, postProcessFlags);
		if (!scene || scene->mNumAnimations == 0)
		{
			MONA_LOG_ERROR("AnimationClip Error: Failed to open file with path {0}", filePath);
			return;
		}

		//Solo cargamos la primera animación
		aiAnimation* animation = scene->mAnimations[0];
		// Dado que los tiempos de muestreo de assimp estan generalmente en ticks se debe dividir casi todos 
		// los tiempos de sus objetos por tickspersecond. Por otro lado, assimp sigue la convencion de que si mTicksPerSecond 
		// es 0.0 entonces los timeStamps de las muestras de las animaciones estan en segundos y no ticks.
		float ticksPerSecond = animation->mTicksPerSecond != 0.0f ? animation->mTicksPerSecond : 1.0f;
		m_duration = animation->mDuration / animation->mTicksPerSecond;
		m_animationTracks.resize(animation->mNumChannels);
		m_trackJointNames.reserve(animation->mNumChannels);
		for (uint32_t i = 0; i < animation->mNumChannels; i++) {
			aiNodeAnim* track = animation->mChannels[i];
			m_trackJointNames.push_back(track->mNodeName.C_Str());
			AnimationClip::AnimationTrack& animationTrack = m_animationTracks[i];
			animationTrack.positions.reserve(track->mNumPositionKeys);
			animationTrack.positionTimeStamps.reserve(track->mNumPositionKeys);
			animationTrack.rotations.reserve(track->mNumRotationKeys);
			animationTrack.rotationTimeStamps.reserve(track->mNumRotationKeys);
			animationTrack.scales.reserve(track->mNumScalingKeys);
			animationTrack.scaleTimeStamps.reserve(track->mNumScalingKeys);
			for (uint32_t j = 0; j < track->mNumPositionKeys; j++) {
				animationTrack.positionTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.positions.push_back(AssimpToGlmVec3(track->mPositionKeys[j].mValue));
			}
			for (uint32_t j = 0; j < track->mNumRotationKeys; j++) {
				animationTrack.rotationTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.rotations.push_back(AssimpToGlmQuat(track->mRotationKeys[j].mValue));

			}
			for (uint32_t j = 0; j < track->mNumScalingKeys; j++) {
				animationTrack.scaleTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.scales.push_back(AssimpToGlmVec3(track->mScalingKeys[j].mValue));

			}
		}


		m_trackJointIndices.resize(m_trackJointNames.size());
		SetSkeleton(skeleton);
		if (removeRootMotion)
			RemoveRootMotion();
	}

	float AnimationClip::Sample(std::vector<JointPose>& outPose, float time, bool isLooping) {
		//Primero se obtiene el tiempo de muestreo correcto
		float newTime = GetSamplingTime(time, isLooping);

		//Por cada articulación o joint animada
		for (uint32_t i = 0; i < m_animationTracks.size(); i++)
		{
			const AnimationTrack& animationTrack = m_animationTracks[i];
			uint32_t jointIndex = m_trackJointIndices[i];
			std::pair<uint32_t, float> fp;
			glm::vec3 localPosition;
			/* Siempre se chequea si hay solo una muestra en el track dado que en ese caso no tiene sentido interpolar.
			En cada cado (rotación, translación y escala) primero se llama una función que obtiene, dado el tiempo de muestreo, 
			el primer indice cuyo tiempo de muestreo es mayor al obtenido y un valor entre 0 y 1 que representa que tan cerca
			esta de dicha muestra, este valor se usa para interpolar.*/
			if (animationTrack.positions.size() > 1)
			{
				
				fp = GetTimeFraction(animationTrack.positionTimeStamps, newTime);
				const glm::vec3& position = animationTrack.positions[fp.first - 1];
				const glm::vec3& nextPosition = animationTrack.positions[fp.first % animationTrack.positions.size()];
				localPosition = glm::mix(position, nextPosition, fp.second);
			}
			else {
				localPosition = animationTrack.positions[0];
			}

			glm::fquat localRotation;
			if (animationTrack.rotations.size() > 1)
			{
				fp = GetTimeFraction(animationTrack.rotationTimeStamps, newTime);
				const glm::fquat& rotation = animationTrack.rotations[fp.first - 1];
				const glm::fquat& nextRotation = animationTrack.rotations[fp.first % animationTrack.rotations.size()];
				//Para interpolar rotaciones se usa slerp en vez de mix (linear interpolation)
				localRotation = glm::slerp(rotation, nextRotation, fp.second);
			}
			else {
				localRotation = animationTrack.rotations[0];
			}

			glm::vec3 localScale;
			if (animationTrack.scales.size() > 1) {
				fp = GetTimeFraction(animationTrack.scaleTimeStamps, newTime);
				const glm::vec3& scale = animationTrack.scales[fp.first - 1];
				const glm::vec3& nextScale = animationTrack.scales[fp.first % animationTrack.scales.size()];
				localScale = glm::mix(scale, nextScale, fp.second);
			}
			else {
				localScale = animationTrack.scales[0];
			}

			outPose[jointIndex] = JointPose(localRotation, localPosition, localScale);

		}
		return newTime;
	}

	void AnimationClip::SetSkeleton(std::shared_ptr<Skeleton> skeletonPtr) {
		//Al momento de configurar el esqueleto se reccorren los tracks para obtener los indices de las articulaciones,
		// dentro del esqueleto recien configurado. De esta forma, dentro del main-loop se usaran indices a arreglos.
		for (uint32_t i = 0; i < m_trackJointNames.size(); i++) {
			const std::string& name = m_trackJointNames[i];
			int32_t signIndex = skeletonPtr->GetJointIndex(name);
			MONA_ASSERT(signIndex >= 0, "AnimationClip Error: Given skeleton incompatible with importing animation");
			uint32_t jointIndex = static_cast<uint32_t>(signIndex);
			m_trackJointIndices[i] = jointIndex;
		}
		m_skeletonPtr = skeletonPtr;
	}

	void AnimationClip::RemoveRootMotion() {
		//Remueve las translaciones del track de animación asociado a la raiz del esqueleto
		for (uint32_t i = 0; i < m_animationTracks.size(); i++)
		{
			AnimationTrack& animationTrack = m_animationTracks[i];
			uint32_t jointIndex = m_trackJointIndices[i];
			if (jointIndex != 0) continue;
			for (uint32_t j = 0; j < animationTrack.positions.size(); j++)
			{
				animationTrack.positions[j] = glm::vec3(0.0f);
			}
		}
	}

	float AnimationClip::GetSamplingTime(float time, bool isLooping) const {
		if (isLooping)
			return std::fmod(time, m_duration);
		return std::clamp(time, 0.0f, m_duration);
	}

	std::pair<uint32_t, float> AnimationClip::GetTimeFraction(const std::vector<float>& timeStamps, float time) const {
		uint32_t sample = 0;
		bool outOfRange = false;
		// Se avanza el indice hasta que el tiempo que se busca es menor al timeStamp actual
		// con esto se sabe que la muestra esta entre este indice y el anterior.
		while (!outOfRange && time >= timeStamps[sample]) {
			sample++;
			outOfRange = sample >= timeStamps.size();
		}

		float start = timeStamps[sample - 1];
		float end = outOfRange ? m_duration : timeStamps[sample];
		float frac = (time - start) / (end - start);
		return { sample, frac };
	}


}