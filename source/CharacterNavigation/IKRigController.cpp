#include "IKRigController.hpp"
#include "../Core/FuncUtils.hpp"

namespace Mona {


	IKRigController::IKRigController(AnimationController* animController, IKRig ikRig): m_animationController(animController), m_ikRig(ikRig) {

	}

	void IKRigController::addAnimation(std::shared_ptr<AnimationClip> animationClip) {
		if (animationClip->GetSkeleton() != m_ikRig.m_skeleton) {
			MONA_LOG_ERROR("IKRig: Input animation does not correspond to base skeleton.");
			return;
		}
		if (!animationClip->m_stableRotations) {
			MONA_LOG_ERROR("IKRig: Animation must have stable rotations (fixed scales and positions per joint).");
			return;
		}
		for (int i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			if (m_ikRig.m_animationConfigs[i].m_animationClip->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added",
					animationClip->GetAnimationName(), m_ikRig.m_skeleton->GetModelName());
				return;
			}
		}

		// Primero guardamos la informacion de traslacion y rotacion de la cadera que sera eliminada mas adelante
		auto hipTrack = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[m_ikRig.m_hipJoint]];
		std::vector<glm::vec3> hipRotAxes(hipTrack.rotations.size());
		std::vector<glm::vec1> hipRotAngles(hipTrack.rotations.size());

		for (int i = 0; i < hipTrack.rotations.size(); i++) {
			hipRotAxes[i] = glm::axis(hipTrack.rotations[i]);
			hipRotAngles[i] = glm::vec1(glm::angle(hipTrack.rotations[i]));
		}

		// Descomprimimos las rotaciones de la animacion, repitiendo valores para que todas las articulaciones 
		// tengan el mismo numero de rotaciones
		std::vector<AnimationClip::AnimationTrack>& tracks = animationClip->m_animationTracks;
		int nTracks = tracks.size();
		std::vector<bool> conditions(nTracks);
		std::vector<int> currentTimeIndexes(nTracks);
		std::vector<float> currentTimes(nTracks);
		for (int i = 0; i < nTracks; i++) {
			currentTimeIndexes[i] = 0;
			conditions[i] = currentTimeIndexes[i] < tracks[i].rotationTimeStamps.size();
		}
		while (funcUtils::conditionArray_OR(conditions)) {
			// seteamos el valor del timestamp que le corresponde a cada track
			for (int i = 0; i < nTracks; i++) {
				currentTimes[i] = conditions[i] ? tracks[i].rotationTimeStamps[currentTimeIndexes[i]] : std::numeric_limits<float>::max();
			}
			// encontramos los indices de las tracks que tienen el minimo timestamp actual
			std::vector<int> minTimeIndexes = funcUtils::minValueIndex_multiple<float>(currentTimes); // ordenados ascendentemente
			float currentMinTime = currentTimes[minTimeIndexes[0]];

			int minTimeIndexesIndex = 0;
			for (int i = 0; i < nTracks; i++) {
				if (minTimeIndexesIndex < minTimeIndexes.size() && minTimeIndexes[minTimeIndexesIndex] == i) { // track actual tiene un timestamp minimo
					currentTimeIndexes[i] += 1;
					minTimeIndexesIndex += 1;
				}
				else {
					// si el valor a insertar cae antes del primer timestamp, se replica el ultimo valor del arreglo de rotaciones
					// se asume animacion circular
					int insertOffset = currentTimeIndexes[i];
					int valIndex = currentTimeIndexes[i] > 0 ? currentTimeIndexes[i] - 1 : tracks[i].rotationTimeStamps.size() - 1;
					auto rotIt = tracks[i].rotations.begin() + insertOffset;
					auto timeRotIt = tracks[i].rotationTimeStamps.begin() + insertOffset;
					tracks[i].rotations.insert(rotIt, tracks[i].rotations[valIndex]);
					tracks[i].rotationTimeStamps.insert(timeRotIt, currentMinTime);
					currentTimeIndexes[i] += 1;
				}
			}

			// actualizamos las condiciones
			for (int i = 0; i < nTracks; i++) {
				conditions[i] = currentTimeIndexes[i] < tracks[i].rotationTimeStamps.size();
			}
		}

		AnimationIndex newIndex = m_ikRig.m_animationConfigs.size();
		m_ikRig.m_animationConfigs.push_back(IKRigConfig(animationClip, newIndex, &m_ikRig.m_forwardKinematics));
		IKRigConfig* currentConfig = m_ikRig.getAnimationConfig(newIndex);


		// Ahora guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();
		float minDistance = m_ikRig.m_rigHeight / 1000;
		std::vector<float> rotTimeStamps = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<glm::vec3>> curvePointsPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<float>> timeStampsPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<bool>> supportFramesPerChain(m_ikRig.m_ikChains.size());
		std::vector<glm::vec3> positions = currentConfig->getBaseModelSpacePositions(0);
		std::vector<glm::vec3> previousPositions(positions.size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::min()));
		for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
			curvePointsPerChain[j].reserve(frameNum);
			timeStampsPerChain[j].reserve(frameNum);
			supportFramesPerChain[j].reserve(frameNum);
		}
		for (int i = 0; i < frameNum; i++) {
			positions = currentConfig->getModelSpacePositions(i, false);
			for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
				int eeIndex = m_ikRig.m_ikChains[j].getJoints().back();
				bool isSupportFrame = glm::distance(positions[eeIndex], previousPositions[eeIndex]) <= minDistance;
				supportFramesPerChain[j].push_back(isSupportFrame);
				if (!isSupportFrame) { // si es suficientemente distinto al anterior, lo guardamos como parte de la curva
					curvePointsPerChain[j].push_back(positions[eeIndex]);
					timeStampsPerChain[j].push_back(rotTimeStamps[i]);
				}
			}
			previousPositions = positions;
		}
		// a los valores de soporte del primer frame les asignamos el valor del ultimo asumiento circularidad
		for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
			supportFramesPerChain[j][0] = supportFramesPerChain[j].back();
		}
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			currentConfig->m_ikChainTrajectoryData[i].supportFrames = supportFramesPerChain[i];
			currentConfig->m_ikChainTrajectoryData[i].originalGlblTrajectory = LIC<3>(curvePointsPerChain[i], timeStampsPerChain[i]);
			currentConfig->m_ikChainTrajectoryData[i].originalGlblTrajectory.scale(glm::vec3(m_ikRig.m_scale));
		}

		currentConfig->m_hipTrajectoryData.originalRotationAngles = LIC<1>(hipRotAngles, hipTrack.rotationTimeStamps);
		currentConfig->m_hipTrajectoryData.originalRotationAxes = LIC<3>(hipRotAxes, hipTrack.rotationTimeStamps);
		currentConfig->m_hipTrajectoryData.originalGlblTranslations = LIC<3>(hipTrack.positions, hipTrack.positionTimeStamps);
		currentConfig->m_hipTrajectoryData.originalGlblTranslations.scale(glm::vec3(m_ikRig.m_scale));
		currentConfig->m_hipTrajectoryData.originalForwardDirection = glm::normalize(hipTrack.positions.back() - hipTrack.positions[0]);

		// Se remueve el movimiento de las caderas
		animationClip->RemoveJointRotation(m_ikRig.m_hipJoint);
		animationClip->RemoveJointTranslation(m_ikRig.m_hipJoint);
		for (int i = 0; i < currentConfig->m_baseJointRotations[m_ikRig.m_hipJoint].size(); i++) {
			currentConfig->m_baseJointRotations[m_ikRig.m_hipJoint][i] = JointRotation(glm::identity<glm::fquat>());
		}

	}

	int IKRigController::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		for (int i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			if (m_ikRig.m_animationConfigs[i].m_animationClip == animationClip) {
				m_ikRig.m_animationConfigs.erase(m_ikRig.m_animationConfigs.begin() + i);
				return i;
			}
		}
		return -1;
	}

	void IKRigController::updateTrajectories(AnimationIndex animIndex) {
		// recalcular trayectorias de ee y caderas

	}
	void IKRigController::updateAnimation(AnimationIndex animIndex) {
		// calcular nuevas rotaciones para la animacion con ik
		// guardar posiciones globales de ee's y caderas generadas

	}

	void IKRigController::updateIKRigConfigTime(float time, AnimationIndex animIndex) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		auto anim = config.m_animationClip;
		float samplingTime = anim->GetSamplingTime(time, true);
		config.m_currentTime = samplingTime;
		int savedCurrentFrameVal = config.m_currentFrameIndex;
		int savedNextFrameVal = config.m_nextFrameIndex;
		for (int i = 0; i < config.m_timeStamps.size(); i++) {
			if (config.m_timeStamps[i] <= samplingTime) {
				config.m_currentFrameIndex = i;
				config.m_nextFrameIndex = (config.m_timeStamps.size()) % (i + 1);
				config.m_requiresIKUpdate = config.m_nextFrameIndex != savedNextFrameVal;
				break;
			}
		}
		if (config.m_requiresIKUpdate) {			
			// si empezamos una nueva vuelta a la animacion
			if (savedCurrentFrameVal == (config.m_timeStamps.size()-1) && config.m_currentFrameIndex == 0) {
				config.m_reproductionCount += 1;
			}
		}
	}

	void IKRigController::updateIKRig(float time) {
		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			updateIKRigConfigTime(time, i);
		}


	}









}