#include "AnimationClip.hpp"
#include <algorithm>
#include <math.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "../Core/AssimpTransformations.hpp"
#include "../Core/Log.hpp"
#include "Skeleton.hpp"
#include "../Core/FuncUtils.hpp"
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

		//Solo cargamos la primera animaci�n
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
				animationTrack.rotationTimeStamps.push_back(track->mRotationKeys[j].mTime / ticksPerSecond);
				animationTrack.rotations.push_back(AssimpToGlmQuat(track->mRotationKeys[j].mValue));

			}
			for (uint32_t j = 0; j < track->mNumScalingKeys; j++) {
				animationTrack.scaleTimeStamps.push_back(track->mScalingKeys[j].mTime / ticksPerSecond);
				animationTrack.scales.push_back(AssimpToGlmVec3(track->mScalingKeys[j].mValue));

			}
		}

		m_trackJointIndices.resize(m_trackJointNames.size());
		SetSkeleton(skeleton);
		if (removeRootMotion) {
			RemoveRootMotion();
		}

		// Se guarda el nombre de la animacion
		size_t pos = filePath.find_last_of("/\\");
		std::string fileName = pos != std::string::npos ? filePath.substr(pos + 1) : filePath;	
		m_animationName = funcUtils::splitString(fileName, '.')[0];
	}

	float AnimationClip::Sample(std::vector<JointPose>& outPose, float time, bool isLooping) {
		//Primero se obtiene el tiempo de muestreo correcto
		float newTime = GetSamplingTime(time, isLooping);

		//Por cada articulaci�n o joint animada
		for (uint32_t i = 0; i < m_animationTracks.size(); i++)
		{
			const AnimationTrack& animationTrack = m_animationTracks[i];
			uint32_t jointIndex = m_trackJointIndices[i];
			std::pair<uint32_t, float> fp;
			glm::vec3 localPosition;
			/* Siempre se chequea si hay solo una muestra en el track dado que en ese caso no tiene sentido interpolar.
			En cada cado (rotaci�n, translaci�n y escala) primero se llama una funci�n que obtiene, dado el tiempo de muestreo, 
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
		//Remueve las translaciones del track de animaci�n asociado a la raiz del esqueleto
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

	void AnimationClip::RemoveJointTranslation(int jointIndex) {
		//Remueve las translaciones del track de animacion asociado a una articulacion del esqueleto
		int trackIndex = GetTrackIndex(jointIndex);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		auto& track = m_animationTracks[trackIndex];
		for (int i = 0; i < track.positions.size(); i++) {
			track.positions[i] = glm::vec3(0);
		}
	}

	void AnimationClip::RemoveJointScaling(int jointIndex) {
		//Remueve los escalamientos del track de animacion asociado a una articulacion del esqueleto
		int trackIndex = GetTrackIndex(jointIndex);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		auto& track = m_animationTracks[trackIndex];
		for (int i = 0; i < track.positions.size(); i++) {
			track.scales[i] = glm::vec3(1);
		}
	}

	void AnimationClip::RemoveJointRotation(int jointIndex) {
		//Remueve las translaciones del track de animacion asociado a una articulacion del esqueleto
		int trackIndex = GetTrackIndex(jointIndex);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		auto& track = m_animationTracks[trackIndex];
		for (int i = 0; i < track.positions.size(); i++) {
			track.rotations[i] = glm::identity<glm::fquat>();
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

	glm::vec3 AnimationClip::GetPosition(float time, int joint, bool isLooping) {
		//Primero se obtiene el tiempo de muestreo correcto
		float newTime = GetSamplingTime(time, isLooping);
		int trackIndex = GetTrackIndex(joint);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		const AnimationTrack& animationTrack = m_animationTracks[trackIndex];
		glm::vec3 localPosition;
		if (animationTrack.positions.size() > 1)
		{

			std::pair<uint32_t, float> fp = GetTimeFraction(animationTrack.positionTimeStamps, newTime);
			const glm::vec3& position = animationTrack.positions[fp.first - 1];
			const glm::vec3& nextPosition = animationTrack.positions[fp.first % animationTrack.positions.size()];
			localPosition = glm::mix(position, nextPosition, fp.second);
		}
		else {
			localPosition = animationTrack.positions[0];
		}
		return localPosition;
	}
	glm::fquat AnimationClip::GetRotation(float time, int joint, bool isLooping) {
		//Primero se obtiene el tiempo de muestreo correcto
		float newTime = GetSamplingTime(time, isLooping);
		int trackIndex = GetTrackIndex(joint);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		const AnimationTrack& animationTrack = m_animationTracks[trackIndex];
		glm::fquat localRotation;
		if (animationTrack.rotations.size() > 1)
		{

			std::pair<uint32_t, float> fp = GetTimeFraction(animationTrack.rotationTimeStamps, newTime);
			const glm::fquat& rotation = animationTrack.rotations[fp.first - 1];
			const glm::fquat& nextRotation = animationTrack.rotations[fp.first % animationTrack.rotations.size()];
			localRotation = glm::mix(rotation, nextRotation, fp.second);
		}
		else {
			localRotation = animationTrack.rotations[0];
		}
		return localRotation;
	}
	glm::vec3 AnimationClip::GetScale(float time, int joint, bool isLooping) {
		//Primero se obtiene el tiempo de muestreo correcto
		float newTime = GetSamplingTime(time, isLooping);
		int trackIndex = GetTrackIndex(joint);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		const AnimationTrack& animationTrack = m_animationTracks[trackIndex];
		glm::vec3 localScale;
		if (animationTrack.scales.size() > 1)
		{

			std::pair<uint32_t, float> fp = GetTimeFraction(animationTrack.scaleTimeStamps, newTime);
			const glm::vec3& scale = animationTrack.scales[fp.first - 1];
			const glm::vec3& nextScale = animationTrack.scales[fp.first % animationTrack.scales.size()];
			localScale = glm::mix(scale, nextScale, fp.second);
		}
		else {
			localScale = animationTrack.scales[0];
		}
		return localScale;
	}

	void AnimationClip::SetRotation(glm::fquat newRotation, int frameIndex, int joint) {
		int trackIndex = GetTrackIndex(joint);
		MONA_ASSERT(trackIndex != -1, "AnimationClip: Joint not present in animation.");
		AnimationTrack& animationTrack = m_animationTracks[trackIndex];
		MONA_ASSERT(0 <= frameIndex && frameIndex < animationTrack.rotations.size(), "AnimationClip: frame index out of range.");
		animationTrack.rotations[frameIndex] = newRotation;
	}

	int AnimationClip::GetTrackIndex(int jointIndex) {
		for (int i = 0; i < m_trackJointIndices.size(); i++) {
			if (m_trackJointIndices[i] == jointIndex) {
				return i;
			}
		}
		return -1;
	}

	void AnimationClip::DecompressRotations() {
		int nTracks = m_animationTracks.size();
		std::vector<bool> conditions(nTracks);
		std::vector<int> currentTimeIndexes(nTracks);
		std::vector<float> currentTimes(nTracks);
		for (int i = 0; i < nTracks; i++) {
			currentTimeIndexes[i] = 0;
			conditions[i] = currentTimeIndexes[i] < m_animationTracks[i].rotationTimeStamps.size();
		}
		while (funcUtils::conditionVector_OR(conditions)) {
			// seteamos el valor del timestamp que le corresponde a cada track
			for (int i = 0; i < nTracks; i++) {
				currentTimes[i] = conditions[i] ? m_animationTracks[i].rotationTimeStamps[currentTimeIndexes[i]] : std::numeric_limits<float>::max();
			}
			// encontramos los indices de las tracks que tienen el minimo timestamp actual
			std::vector<int> minTimeIndexes = funcUtils::minValueIndex_multiple<float>(currentTimes); // ordenados ascendentemente
			float currentMinTime = currentTimes[minTimeIndexes[0]];

			int minTimeIndexesIndex = 0;
			for (int i = 0; i < nTracks; i++) {
				if (minTimeIndexesIndex < minTimeIndexes.size() && minTimeIndexes[minTimeIndexesIndex] == i) { // track actual tiene un timestamp minimo
					minTimeIndexesIndex += 1;
				}
				else {
					// si el valor a insertar cae antes del primer timestamp, se replica el ultimo valor del arreglo de rotaciones
					// se asume animacion circular
					int insertOffset = currentTimeIndexes[i];
					int valIndex = currentTimeIndexes[i] > 0 ? currentTimeIndexes[i] - 1 : m_animationTracks[i].rotationTimeStamps.size() - 1;
					auto rotIt = m_animationTracks[i].rotations.begin() + insertOffset;
					auto timeRotIt = m_animationTracks[i].rotationTimeStamps.begin() + insertOffset;
					glm::fquat rotVal = m_animationTracks[i].rotations[valIndex];
					if (valIndex < m_animationTracks[i].rotations.size() - 1) {
						float timeFrac = funcUtils::getFraction(m_animationTracks[i].rotationTimeStamps[valIndex],
							m_animationTracks[i].rotationTimeStamps[valIndex + 1], currentMinTime);
						rotVal = funcUtils::lerp(rotVal, m_animationTracks[i].rotations[valIndex + 1], timeFrac);
					}
					m_animationTracks[i].rotations.insert(rotIt, rotVal);
					m_animationTracks[i].rotationTimeStamps.insert(timeRotIt, currentMinTime);
				}
				currentTimeIndexes[i] += 1;
			}

			// actualizamos las condiciones
			for (int i = 0; i < nTracks; i++) {
				conditions[i] = currentTimeIndexes[i] < m_animationTracks[i].rotationTimeStamps.size();
			}
		}
	}


	void AnimationClip::Rotate(glm::fquat rotation) {
		AnimationTrack& rootTrack = m_animationTracks[GetTrackIndex(0)];
		for (int i = 0; i < rootTrack.rotations.size(); i++) {
			rootTrack.rotations[i] = rotation * rootTrack.rotations[i];
		}
		for (int i = 0; i < rootTrack.positions.size(); i++) {
			rootTrack.positions[i] = rotation * rootTrack.positions[i];
		}
	}
	void AnimationClip::Scale(float scale) {
		AnimationTrack& rootTrack = m_animationTracks[GetTrackIndex(0)];
		for (int i = 0; i < rootTrack.scales.size(); i++) {
			rootTrack.scales[i] = scale * rootTrack.scales[i];
		}
		for (int i = 0; i < rootTrack.positions.size(); i++) {
			rootTrack.positions[i] = scale * rootTrack.positions[i];
		}
	}
	void AnimationClip::Translate(glm::vec3 translation) {
		AnimationTrack& rootTrack = m_animationTracks[GetTrackIndex(0)];
		for (int i = 0; i < rootTrack.positions.size(); i++) {
			rootTrack.positions[i] = translation + rootTrack.positions[i];
		}
	}

}