#include "AnimationClip.hpp"
#include <algorithm>
#include <math.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "../Core/AssimpTransformations.hpp"
#include "../Core/Log.hpp"
#include "Skeleton.hpp"
#include "../Utilities/FuncUtils.hpp"
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

			// regularizamos transformaciones rellenando con la identidad donde haga falta
			animationTrack.positions.reserve(track->mNumPositionKeys);
			animationTrack.positionTimeStamps.reserve(track->mNumPositionKeys);
			animationTrack.rotations.reserve(track->mNumRotationKeys);
			animationTrack.rotationTimeStamps.reserve(track->mNumRotationKeys);
			animationTrack.stableRotations.reserve(track->mNumRotationKeys);
			animationTrack.scales.reserve(track->mNumScalingKeys);
			animationTrack.scaleTimeStamps.reserve(track->mNumScalingKeys);
			int posIndex = 0;
			int rotIndex = 0;
			int sclIndex = 0;
			while (posIndex < track->mNumPositionKeys || rotIndex < track->mNumRotationKeys || sclIndex < track->mNumScalingKeys) {
				aiVector3D currPos = posIndex < track->mNumPositionKeys ? track->mPositionKeys[posIndex].mValue : aiVector3D({0,0,0});
				aiQuaternion currRot = rotIndex < track->mNumRotationKeys ? track->mRotationKeys[rotIndex].mValue : aiQuaternion({ 1,0,0,0 });
				aiVector3D currScl = sclIndex < track->mNumScalingKeys ? track->mScalingKeys[sclIndex].mValue : aiVector3D({ 1,1,1 });
				double currPosTime = posIndex < track->mNumPositionKeys ? track->mPositionKeys[posIndex].mTime / ticksPerSecond : std::numeric_limits<double>::max();
				double currRotTime = rotIndex < track->mNumRotationKeys ? track->mRotationKeys[rotIndex].mTime / ticksPerSecond : std::numeric_limits<double>::max();
				double currSclTime = sclIndex < track->mNumScalingKeys ? track->mScalingKeys[sclIndex].mTime / ticksPerSecond : std::numeric_limits<double>::max();
				std::vector<double> times = { currPosTime, currRotTime, currSclTime };
				std::vector<int> minIndices = funcUtils::minValueIndex_multiple<double>(times);
				if (minIndices.size() == 3) {
					posIndex += 1;
					rotIndex += 1;
					sclIndex += 1;
				}
				else if(minIndices.size()==2) {
					if (minIndices[0] == 0 && minIndices[1] == 1) {
						currSclTime = currPosTime;
						currScl = aiVector3D({ 1,1,1 });
						posIndex += 1;
						rotIndex += 1;
					}
					else if (minIndices[0] == 1 && minIndices[1] == 2) {
						currPosTime = currSclTime;
						currPos = aiVector3D({ 0,0,0 });
						rotIndex += 1;
						sclIndex += 1;
					}
					else if (minIndices[0] == 0 && minIndices[1] == 2) {
						currRotTime = currSclTime;
						currRot = aiQuaternion({ 1,0,0,0 });
						posIndex += 1;
						sclIndex += 1;
					}
				}
				else if (minIndices.size() == 1) {
					if (minIndices[0] == 0) {
						currRotTime = currPosTime;
						currRot = aiQuaternion({ 1,0,0,0 });
						currSclTime = currPosTime;
						currScl = aiVector3D({ 1,1,1 });
					}
					else if (minIndices[0] == 1) {
						currPosTime = currRotTime;
						currPos = aiVector3D({ 0,0,0 });
						currSclTime = currRotTime;
						currScl = aiVector3D({ 1,1,1 });
					}
					else if (minIndices[0] == 2) {
						currPosTime = currSclTime;
						currPos = aiVector3D({ 0,0,0 });
						currRotTime = currSclTime;
						currRot = aiQuaternion({ 1,0,0,0 });
					}
				}

				animationTrack.positionTimeStamps.push_back(currPosTime);
				animationTrack.positions.push_back(AssimpToGlmVec3(currPos));
				animationTrack.rotationTimeStamps.push_back(currRotTime);
				animationTrack.rotations.push_back(AssimpToGlmQuat(currRot));
				animationTrack.scaleTimeStamps.push_back(currSclTime);
				animationTrack.scales.push_back(AssimpToGlmVec3(currScl));
				

			}
		}

		// luego se modifican los tracks de ser encesario para que todos los tracks tengan el mismo numero de frames en los mismos tiempos
		std::vector<int> timeIndexes(m_animationTracks.size());
		std::vector<bool> conditions(m_animationTracks.size());
		for (int i = 0; i < timeIndexes.size(); i++) { timeIndexes[i] = 0; }
		for (int i = 0; i < conditions.size(); i++) { conditions[i] = true; }
		while (funcUtils::conditionArray_OR(conditions)) {
			std::vector<float> currentTimes;
			std::vector<glm::vec3> currentPositions;
			std::vector<glm::fquat> currentRotations;
			std::vector<glm::vec3> currentScales;
			for (int i = 0; i < m_animationTracks.size(); i++) {
				float currTime = timeIndexes[i] < m_animationTracks[i].rotationTimeStamps.size() ? m_animationTracks[i].rotationTimeStamps[timeIndexes[i]] : std::numeric_limits<float>::max();
				glm::vec3 currPos = timeIndexes[i] < m_animationTracks[i].positions.size() ? m_animationTracks[i].positions[timeIndexes[i]] : glm::vec3({ 0,0,0 });
				glm::fquat currRot = timeIndexes[i] < m_animationTracks[i].rotations.size() ? m_animationTracks[i].rotations[timeIndexes[i]] : glm::fquat({ 1,0,0,0 });
				glm::vec3 currScl = timeIndexes[i] < m_animationTracks[i].scales.size() ? m_animationTracks[i].scales[timeIndexes[i]] : glm::vec3({ 1,1,1 });
				currentTimes.push_back(currTime);
				currentPositions.push_back(currPos);
				currentRotations.push_back(currRot);
				currentScales.push_back(currScl);
			}
			std::vector<int> minIndexes = funcUtils::minValueIndex_multiple<float>(currentTimes);
			float currMinTime = currentTimes[minIndexes[0]];
			glm::vec3 currMinPos = currentPositions[minIndexes[0]];
			glm::fquat currMinRot = currentRotations[minIndexes[0]];
			glm::vec3 currMinScl = currentScales[minIndexes[0]];
			int currMinIndexes_idx = minIndexes[0];
			for (int i = 0; i < m_animationTracks.size(); i++) {
				if (currMinIndexes_idx < minIndexes.size() && i == minIndexes[currMinIndexes_idx]) { // esta entre las animaciones con valor minimo
					currMinIndexes_idx += 1;
					timeIndexes[i] += 1;
				}
				else {
					auto insertIndPos = m_animationTracks[i].positions.begin() + std::max(timeIndexes[i] - 1, 0); // se insertan los nuevos valores antes de la posicion minima acutal del track
					auto insertIndRot = m_animationTracks[i].rotations.begin() + std::max(timeIndexes[i] - 1, 0);
					auto insertIndScl = m_animationTracks[i].scales.begin() + std::max(timeIndexes[i] - 1, 0);
					auto insertIndTime = m_animationTracks[i].positionTimeStamps.begin() + std::max(timeIndexes[i] - 1, 0);
					m_animationTracks[i].positions.insert(insertIndPos, currMinPos);
					m_animationTracks[i].rotations.insert(insertIndRot, currMinRot);
					m_animationTracks[i].scales.insert(insertIndScl, currMinScl);
					m_animationTracks[i].positionTimeStamps.insert(insertIndTime, currMinTime);
					m_animationTracks[i].rotationTimeStamps.insert(insertIndTime, currMinTime);
					m_animationTracks[i].scaleTimeStamps.insert(insertIndTime, currMinTime);
					timeIndexes[i] += 1; // se saltan los valores recien insertados
				}
			}
			// update conditions
			for (int i = 0; i < m_animationTracks.size(); i++) {
				if (!timeIndexes[i] < m_animationTracks[i].positionTimeStamps.size()) {
					conditions[i] = false;
				}
			}
		}

		m_trackJointIndices.resize(m_trackJointNames.size());
		SetSkeleton(skeleton);
		if (removeRootMotion)
			RemoveRootMotion();


		// Calcular rotaciones estabilizadas
		for (int i = 0; i < m_animationTracks.size(); i++) {
			AnimationTrack& track = m_animationTracks[i];
			int maxIndex = std::max(std::max(track.positions.size(), track.rotations.size()), track.scales.size());
			for (int j = 0; j < maxIndex; j++) {
				glm::vec3 scl = j < track.scales.size() ? track.scales[j] :	glm::vec3({ 1,1,1 });
				glm::vec3 tr = j < track.positions.size() ? track.positions[j] : glm::vec3({ 0,0,0 });
				glm::fquat rot = j < track.rotations.size() ? track.rotations[j] : glm::fquat({1,0,0,0 });
				glm::mat4 mat = glm::identity<glm::mat4>();
				mat = glm::scale(mat, scl);
				mat = glm::toMat4(rot) * mat;
				mat = glm::translate(mat, tr);
				glm::mat4 fromBindPoseMat = mat* glm::inverse(skeleton->m_offsets[m_trackJointIndices[i]]);
				track.stableRotations.push_back(glm::toQuat(fromBindPoseMat));
			}
		}

		// Se guarda el nombre de la animacion
		std::string fileName = filePath;
		size_t pos = filePath.find_last_of("/\\");
		if (pos != std::string::npos) {
			fileName = filePath.substr(pos + 1);
		}		
		m_animationName = funcUtils::splitString(fileName, '.')[0];
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