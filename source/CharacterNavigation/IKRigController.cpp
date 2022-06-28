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
		for (int i = 0; i < m_ikRig.m_animations.size(); i++) {
			if (m_ikRig.m_animations[i]->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added",
					animationClip->GetAnimationName(), m_ikRig.m_skeleton->GetModelName());
				return;
			}
		}
		// descomprimimos las rotaciones de la animacion, repitiendo valores para que todas las articulaciones 
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

		m_ikRig.m_animations.push_back(animationClip);
		m_ikRig.m_animationConfigs.push_back(IKRigConfig(animationClip, m_ikRig.m_animations.size() - 1, &m_ikRig.m_forwardKinematics));


		// Ahora guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();
		float minDistance = m_ikRig.m_rigHeight / 1000;
		std::vector<float> tValues = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<glm::vec3>> splinePointsPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<float>> timeStampsPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<bool>> supportFramesPerChain(m_ikRig.m_ikChains.size());
		std::vector<glm::vec3> positions = m_ikRig.m_animationConfigs.back().getBaseModelSpacePositions(0);
		std::vector<glm::vec3> previousPositions(positions.size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::min()));
		for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
			splinePointsPerChain[j].reserve(frameNum);
			timeStampsPerChain[j].reserve(frameNum);
			supportFramesPerChain[j].reserve(frameNum);
		}
		for (int i = 0; i < frameNum; i++) {
			positions = m_ikRig.m_animationConfigs.back().getBaseModelSpacePositions(i);
			for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
				int eeIndex = m_ikRig.m_ikChains[j].getJoints().back();
				bool isSupportFrame = glm::distance(positions[eeIndex], previousPositions[eeIndex]) <= minDistance;
				supportFramesPerChain[j].push_back(isSupportFrame);
				if (!isSupportFrame) { // si es suficientemente distinto al anterior, lo guardamos como parte de la curva
					splinePointsPerChain[j].push_back(positions[eeIndex]);
					timeStampsPerChain[j].push_back(tValues[i]);
				}
			}
			previousPositions = positions;
		}
		// a los valores de soporte del primer frame les asignamos el valor del ultimo asumiento circularidad
		for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
			supportFramesPerChain[j][0] = supportFramesPerChain[j].back();
		}
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			m_ikRig.m_animationConfigs.back().m_ikChainTrajectoryData[i].eeSupportFrames = supportFramesPerChain[i];
			m_ikRig.m_animationConfigs.back().m_ikChainTrajectoryData[i].eeBaseTrajectory = BezierSpline(splinePointsPerChain[i], timeStampsPerChain[i]);
		}

		animationClip->RemoveRootMotion();
	}

	int IKRigController::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		for (int i = 0; i < m_ikRig.m_animations.size(); i++) {
			if (m_ikRig.m_animations[i] == animationClip) {
				m_ikRig.m_animations.erase(m_ikRig.m_animations.begin() + i);
				m_ikRig.m_animationConfigs.erase(m_ikRig.m_animationConfigs.begin() + i);
				return i;
			}
		}
		return -1;
	}

	void IKRigController::updateIKRigConfigTime(float time, AnimationIndex animIndex) {
		auto anim = m_ikRig.m_animations[animIndex];
		auto& config = m_ikRig.m_animationConfigs[animIndex];
		float samplingTime = anim->GetSamplingTime(time, m_animationController->GetIsLooping());
		config.m_currentTime = samplingTime;
		int savedFrameVal = config.m_nextFrameIndex;
		for (int i = 0; i < config.m_timeStamps.size(); i++) {
			if (config.m_timeStamps[i] <= samplingTime) {
				config.m_nextFrameIndex = (config.m_timeStamps.size()) % (i + 1);
				config.m_requiresIKUpdate = config.m_nextFrameIndex != savedFrameVal;
				break;
			}
		}
		if (config.m_requiresIKUpdate) {
			for (int i = 0; i < config.m_dynamicJointRotations[config.m_nextFrameIndex].size(); i++) {
				config.m_dynamicJointRotations[config.m_nextFrameIndex][i] = JointRotation(anim->m_animationTracks[i].rotations[config.m_nextFrameIndex]);
			}
		}
	}

	void IKRigController::updateIKRig(float time) {
		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			updateIKRigConfigTime(time, i);
		}


	}









}