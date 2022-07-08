#include "IKRigController.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Core/GlmUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"

namespace Mona {


	IKRigController::IKRigController(AnimationController* animController, IKRig ikRig): m_animationController(animController), m_ikRig(ikRig) {

	}

	void IKRigController::addAnimation(std::shared_ptr<AnimationClip> animationClip) {
		if (animationClip->GetSkeleton() != m_ikRig.m_skeleton) {
			MONA_LOG_ERROR("IKRig: Input animation does not correspond to base skeleton.");
			return;
		}
		
		// Chequar si los escalamientos y traslaciones son constantes por joint.
		// La cadera si puede tener traslaciones variables
		for (JointIndex i = 0; i < animationClip->m_jointTrackIndices.size(); i++) {
			auto track = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[i]];
			glm::vec3 basePosition = track.positions[0];
			glm::vec3 baseScale = track.scales[0];
			for (int j = 1; j < track.positions.size(); j++) {
				if (track.positions[j] != basePosition && i!=m_ikRig.m_hipJoint) {
					MONA_LOG_ERROR("IKRigController: Animation must have fixed translations per joint).");
					return;
				}
			}
			for (int j = 1; j < track.scales.size(); j++) {
				if (track.scales[j] != baseScale) {
					MONA_LOG_ERROR("IKRigController: Animation must have fixed scales per joint).");
					return;
				}
			}
		}

		JointIndex parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		while (parent != -1) {
			auto track = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[0]];
			glm::fquat baseRotation = track.rotations[0];
			for (int j = 1; j < track.rotations.size(); j++) {
				if (track.rotations[j] != baseRotation) {
					MONA_LOG_ERROR("IKRigController: Joints above the hip in the hierarchy cannot have vatiable rotations.");
					return;
				}
			}
			parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		}

		for (int i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			if (m_ikRig.m_animationConfigs[i].m_animationClip->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added",
					animationClip->GetAnimationName(), m_ikRig.m_skeleton->GetModelName());
				return;
			}
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

		// Para este paso las rotaciones deben estar descomprimidas
		AnimationIndex newIndex = m_ikRig.m_animationConfigs.size();
		m_ikRig.m_animationConfigs.push_back(IKRigConfig(animationClip, newIndex, &m_ikRig.m_forwardKinematics));
		IKRigConfig* currentConfig = m_ikRig.getAnimationConfig(newIndex);

		// Guardamos la informacion de traslacion y rotacion de la cadera, antes de eliminarla
		auto hipTrack = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[m_ikRig.m_hipJoint]];
		std::vector<float> hipTimeStamps = hipTrack.positionTimeStamps;
		std::vector<glm::vec3> hipRotAxes(hipTimeStamps.size());
		std::vector<glm::vec1> hipRotAngles(hipTimeStamps.size());
		std::vector<glm::vec3> hipTranslations(hipTimeStamps.size());
		glm::vec3 hipScale;
		JointIndex hipIndex = m_ikRig.m_hipJoint;
		for (int i = 0; i < hipTimeStamps.size(); i++) {
			float timeStamp = hipTimeStamps[i];
			JointIndex parent = hipIndex;
			glm::mat4 hipTransform = glm::identity<glm::mat4>();
			while (parent != -1) {
				hipTransform *= glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, parent, true)) *
					glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, parent, true)) *
					glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, parent, true));
				parent = m_ikRig.getTopology()[parent];
			}
			glm::vec3 scale;
			glm::quat rotation;
			glm::vec3 translation;
			glm::vec3 skew;
			glm::vec4 perspective;
			glm::decompose(hipTransform, scale, rotation, translation, skew, perspective);
			hipRotAngles[i] = glm::vec1(glm::angle(rotation));
			hipRotAxes[i] = glm::axis(rotation);
			hipTranslations[i] = translation;
			if (i == 0) { hipScale = scale; }
		}

		currentConfig->m_hipTrajectoryData.originalRotationAngles = LIC<1>(hipRotAngles, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.originalRotationAxes = LIC<3>(hipRotAxes, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.originalGlblTranslations = LIC<3>(hipTranslations, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.originalGlblTranslations.scale(glm::vec3(m_ikRig.m_scale));
		currentConfig->m_hipTrajectoryData.originalForwardDirection = glm::normalize(glm::vec2(hipTrack.positions.back()) - 
			glm::vec2(hipTrack.positions[0]));

		// Ahora guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();
		float minDistance = m_ikRig.m_rigHeight / 1000;
		std::vector<float> rotTimeStamps = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<glm::vec3>> curvePointsPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<float>> timeStampsPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<bool>> supportFramesPerChain(m_ikRig.m_ikChains.size());
		std::vector<glm::vec3> positions(m_ikRig.getTopology().size());
		std::vector<glm::mat4> transforms(m_ikRig.getTopology().size());
		std::vector<glm::vec3> previousPositions(m_ikRig.getTopology().size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::min()));
		for (int j = 0; j < m_ikRig.m_ikChains.size(); j++) {
			curvePointsPerChain[j].reserve(frameNum);
			timeStampsPerChain[j].reserve(frameNum);
			supportFramesPerChain[j].reserve(frameNum);
		}
		for (int i = 0; i < frameNum; i++) {
			// calculo de las transformaciones
			for (int j = 0; i < transforms.size(); j++) {
				transforms[j] = glm::identity<glm::mat4>();
			}
			float timeStamp = rotTimeStamps[i];
			transforms[0] = glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, 0, true)) *
				glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, 0, true)) *
				glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, 0, true));
			for (int j = 1; j < m_ikRig.getTopology().size(); j++) {
				transforms[j] = transforms[m_ikRig.getTopology()[j]] *
					glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, j, true)) *
					glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, j, true)) *
					glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, j, true));
			}

			// calculo de las posiciones
			for (int j = 0; j < positions.size(); j++) {
				positions[j] = transforms[j] * glm::vec4(0, 0, 0, 1);
			}
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

		// calculo de la altura maxima para cada trayectoria original de los ee


		

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

	void IKRigController::updateFrontVector(float time) {
		float rotAngle = m_ikRig.m_angularSpeed * time;
		m_ikRig.m_frontVector = glm::rotate(m_ikRig.m_frontVector, rotAngle);
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
		updateFrontVector(time);
	}









}