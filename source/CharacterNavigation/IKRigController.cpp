#include "IKRigController.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Core/GlmUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"

namespace Mona {

	

	IKRigController::IKRigController(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle,
		InnerComponentHandle skeletalMeshHandle, ComponentManager<TransformComponent>* transformManagerPtr):
		m_skeletalMeshHandle(skeletalMeshHandle), m_ikRig(skeleton, rigData, transformHandle) {

		// seteamos la transformacion base
		transformManagerPtr->GetComponentPointer(transformHandle)->SetRotation(glm::identity<glm::fquat>());
		transformManagerPtr->GetComponentPointer(transformHandle)->SetScale(glm::vec3(m_ikRig.m_rigScale));
		transformManagerPtr->GetComponentPointer(transformHandle)->SetTranslation(m_ikRig.m_initialPosition);

		m_ikEnabled = true;
		m_transitioning = false;

	}
	void IKRigController::init() {
		m_ikRig.init();
		m_animationValidator = AnimationValidator(&m_ikRig);
	}

	void IKRigController::validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager) {
		m_ikRig.m_trajectoryGenerator.m_environmentData.validateTerrains(staticMeshManager);
	}

	void IKRigController::addAnimation(std::shared_ptr<AnimationClip> animationClip, glm::vec3 originalUpVector, 
		glm::vec3 originalFrontVector, AnimationType animationType, float supportFrameDistanceFactor) {
		
		m_animationValidator.checkTransforms(animationClip);

		// Transformacion base
		glm::mat4 baseGlobalTransform = glmUtils::translationToMat4(m_ikRig.m_initialPosition)*glmUtils::scaleToMat4(glm::vec3(m_ikRig.m_rigScale));

		// Descomprimimos las rotaciones de la animacion, repitiendo valores para que todas las articulaciones 
		// tengan el mismo numero de rotaciones
		animationClip->DecompressRotations();

		// Para los siguientes pasos las rotaciones deben estar descomprimidas
		m_animationValidator.correctAnimationOrientation(animationClip, originalUpVector, originalFrontVector);
		if (animationType == AnimationType::WALKING) {
			for (int i = 0; i < m_ikRig.getChainNum(); i++) {
					m_animationValidator.checkLegLocalRotationAxes(animationClip, m_ikRig.getIKChain(i), originalUpVector, originalFrontVector);
			}
		}
		AnimationIndex newIndex = m_ikRig.m_ikAnimations.size();
		m_ikRig.m_ikAnimations.push_back(IKAnimation(animationClip, animationType, newIndex, &m_ikRig.m_forwardKinematics));
		IKAnimation* currentIKAnim = m_ikRig.getIKAnimation(newIndex);
		currentIKAnim->m_eeTrajectoryData = std::vector<EEGlobalTrajectoryData>(m_ikRig.m_ikChains.size());
		// numero de rotaciones por joint con la animaciond descomprimida
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();
		int chainNum = m_ikRig.getChainNum();		

		// Guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		std::vector<float> rotTimeStamps = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<bool>> supportFramesPerChain(chainNum);
		std::vector<std::vector<glm::vec3>> glblPositionsPerChain(chainNum);
		std::vector<glm::vec3> glblPositions(m_ikRig.getTopology().size());
		std::vector<glm::mat4> glblTransforms(m_ikRig.getTopology().size());
		std::vector<glm::vec3> hipGlblPositions;
		float floorZ = std::numeric_limits<float>::max(); // altura del piso para la animacion
		for (int i = 0; i < m_ikRig.getTopology().size(); i++) { glblTransforms[i] = glm::identity<glm::mat4>(); }
		std::vector<glm::vec3> previousPositions(m_ikRig.getTopology().size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::lowest()));
		for (ChainIndex i = 0; i < chainNum; i++) {
			supportFramesPerChain[i] = std::vector<bool>(frameNum);
			glblPositionsPerChain[i] = std::vector<glm::vec3>(frameNum);
		}
		for (FrameIndex i = 0; i < frameNum; i++) {
			// calculo de las transformaciones
			float timeStamp = rotTimeStamps[i];
			while (currentIKAnim->getAnimationDuration() <= timeStamp) { timeStamp -= 0.000001; }
			for (int j = 0; j < currentIKAnim->getJointIndices().size(); j++) {
				JointIndex jIndex = currentIKAnim->getJointIndices()[j];
				glm::mat4 baseTransform = j == 0 ? baseGlobalTransform : glblTransforms[m_ikRig.getTopology()[jIndex]];
				glblTransforms[jIndex] = baseTransform *
					glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, jIndex, true)) *
					glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, jIndex, true)) *
					glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, jIndex, true));
			}

			// calculo de las posiciones
			for (int j = 0; j < glblPositions.size(); j++) {
				glblPositions[j] = glblTransforms[j] * glm::vec4(0, 0, 0, 1);
			}
			hipGlblPositions.push_back(glblPositions[m_ikRig.m_hipJoint]);
			// distancia base entre puntos para identificacion de puntos de soporte
			float minDistance = 20*(m_ikRig.m_rigHeight * m_ikRig.m_rigScale / 1000);
			for (ChainIndex j = 0; j < chainNum; j++) {
				int eeIndex = m_ikRig.m_ikChains[j].getEndEffector();
				bool isSupportFrame = glm::distance(glblPositions[eeIndex], previousPositions[eeIndex]) <= minDistance*supportFrameDistanceFactor;
				supportFramesPerChain[j][i] = isSupportFrame;
				glblPositionsPerChain[j][i] = glm::vec4(glblPositions[eeIndex], 1);
			}
			for (int j = 0; j < currentIKAnim->getJointIndices().size(); j++) {
				JointIndex jIndex = currentIKAnim->getJointIndices()[j];
				if (glblPositions[jIndex][2] < floorZ) {
					floorZ = glblPositions[jIndex][2];
				}
			}
			previousPositions = glblPositions;
		}
		// ajuste de las alturas con el suelo
		for (ChainIndex i = 0; i < chainNum; i++) {
			for (FrameIndex j = 0; j < frameNum; j++) {
				glblPositionsPerChain[i][j][2] -= floorZ;
			}
		}
		for (FrameIndex i = 0; i < frameNum; i++) {
			hipGlblPositions[i][2] -= floorZ;
		}

		// Guardamos la informacion de traslacion y rotacion de la cadera, antes de eliminarla
		TrajectoryGenerator::buildHipTrajectory(currentIKAnim, hipGlblPositions);
		
		// si hay un frame que no es de soporte entre dos frames que si lo son, se setea como de soporte
		// si el penultimo es de soporte, tambien se setea el ultimo como de soporte
		// a los valores de soporte del primer frame les asignamos el valor del ultimo asumiento circularidad
		for (int i = 0; i < chainNum; i++) {
			if (supportFramesPerChain[i][frameNum - 2]) {
				supportFramesPerChain[i].back() = true;
			}
			supportFramesPerChain[i][0] = supportFramesPerChain[i].back();
			for (int j = 1; j < frameNum - 1; j++) {
				if (supportFramesPerChain[i][j - 1] && supportFramesPerChain[i][j + 1]) {
					supportFramesPerChain[i][j] = true;
				}
			}			
		}
		for (int i = 0; i < chainNum; i++) {
			supportFramesPerChain[i][0] = supportFramesPerChain[i].back();
		}

		// se ajustan los frames de soporte entre cadenas opuestas (izquierda y derecha)
		std::vector<ChainIndex> pairedChains;
		for (ChainIndex i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			IKChain currChain = m_ikRig.m_ikChains[i];
			if (funcUtils::findIndex(pairedChains, i) == -1) {
				for (FrameIndex j = 0; j < frameNum; j++) {
					supportFramesPerChain[currChain.getOpposite()][j] = !supportFramesPerChain[i][j];
				}
				pairedChains.push_back(i);
				pairedChains.push_back(currChain.getOpposite());
			}
		}

		// dividimos cada trayectoria global (por ee) en sub trayectorias dinamicas y estaticas.
		std::vector<ChainIndex> oppositePerChain;
		for (ChainIndex i = 0; i < chainNum; i++) {
			ChainIndex opposite = m_ikRig.getIKChain(i)->getOpposite();
			oppositePerChain.push_back(opposite);
		}
		TrajectoryGenerator::buildEETrajectories(currentIKAnim, supportFramesPerChain, glblPositionsPerChain, oppositePerChain);

		// Se remueve el movimiento de las caderas
		animationClip->RemoveJointTranslation(m_ikRig.m_hipJoint);
	}

	AnimationIndex IKRigController::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		for (int i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
			if (m_ikRig.m_ikAnimations[i].m_animationClip == animationClip) {
				m_ikRig.resetAnimation(i);
				m_ikRig.m_ikAnimations.erase(m_ikRig.m_ikAnimations.begin() + i);
				return i;
			}
		}
		return -1;
	}

	void IKRigController::updateMovementDirection(float timeStep) {
		m_ikRig.m_rotationAngle += m_ikRig.m_angularSpeed * timeStep;
		m_ikRig.m_rotationAngle = funcUtils::normalizeAngle(m_ikRig.m_rotationAngle);
	}

	void IKRigController::updateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		IKAnimation& ikAnim = m_ikRig.m_ikAnimations[animIndex];
		HipGlobalTrajectoryData* hipTrData = ikAnim.getHipTrajectoryData();
		FrameIndex currentFrame = ikAnim.getCurrentFrameIndex();
		float currentFrameRepTime = ikAnim.getReproductionTime(currentFrame);
		float avgFrameDuration = ikAnim.getAnimationDuration() / ikAnim.getFrameNum();
		EEGlobalTrajectoryData* trData;
		if (ikAnim.m_onNewFrame) { // se realiza al llegar a un frame de la animacion
			// chequear estado de angulos guardados, necesarios para calculo de posiciones globales
			for (int i = 0; i < m_ikRig.getTopology().size(); ++i) {
				if (!ikAnim.m_savedAngles[i].inTRange(currentFrameRepTime)) {
					float lastCalcTime = ikAnim.m_savedAngles[i].getTRange()[1];
					float maxElapsed = avgFrameDuration * 5;
					// si nos saltamos por mucho el ultimo frame calculado reseteamos los angulos o no hay valores
					if (maxElapsed < abs(lastCalcTime - currentFrameRepTime) 
						|| ikAnim.m_savedAngles[i].getNumberOfPoints() == 0
						|| currentFrameRepTime < lastCalcTime) {
						ikAnim.refreshSavedAngles(i);
					}
					else {
						float fraction = funcUtils::getFraction(lastCalcTime, lastCalcTime + maxElapsed, currentFrameRepTime);
						float limitNewAngle = ikAnim.m_savedAngles[i].getEnd()[0] * 0.6f + ikAnim.getOriginalJointRotations(currentFrame)[i].getRotationAngle() * 0.4f;
						float interpolatedAngle = funcUtils::lerp(ikAnim.m_savedAngles[i].getEnd()[0], limitNewAngle, fraction);
						ikAnim.m_savedAngles[i].insertPoint(glm::vec1(interpolatedAngle), currentFrameRepTime);
					}
					
				}
			}

			// guardado de posiciones globales ee y cadera
			std::vector<JointIndex> endEffectors;
			for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
				endEffectors.push_back(m_ikRig.m_ikChains[i].getEndEffector());
			}
			glm::mat4 baseTransform = transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->GetModelMatrix();
			std::vector<glm::mat4> globalTransforms = ikAnim.getEEListCustomSpaceTransforms(endEffectors, baseTransform, currentFrameRepTime);
			for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
				trData = ikAnim.getEETrajectoryData(i);
				JointIndex ee = endEffectors[i];
				glm::vec3 eePos = globalTransforms[ee] * glm::vec4(0, 0, 0, 1);
				// si no hay posiciones guardadas
				if (trData->m_savedPositions.getNumberOfPoints() == 0 || m_transitioning) {
					trData->m_savedPositions = LIC<3>({ eePos, eePos }, { currentFrameRepTime - ikAnim.getAnimationDuration(), currentFrameRepTime });
					trData->m_motionInitialized = false;
				}
				else {
					trData->m_savedPositions.insertPoint(eePos, currentFrameRepTime);
					// recorte de las posiciones guardadas
					for (int j = trData->m_savedPositions.getNumberOfPoints() - 1; 0 <= j; j--) {
						float tVal = trData->m_savedPositions.getTValue(j);
						float minVal = trData->m_savedPositions.getTRange()[1] - ikAnim.getAnimationDuration() * 2;
						if (tVal < minVal) {
							trData->m_savedPositions = trData->m_savedPositions.sample(tVal, trData->m_savedPositions.getTRange()[1]);
							break;
						}
					}
				}
			}
			glm::mat4 hipTransform = globalTransforms[m_ikRig.m_hipJoint];
			glm::vec3 hipScale; glm::fquat hipRot; glm::vec3 hipTrans; glm::vec3 hipSkew; glm::vec4 hipPers;
			glm::decompose(hipTransform, hipScale, hipRot, hipTrans, hipSkew, hipPers);
			
			// si no hay posiciones guardadas
			if (hipTrData->m_savedPositions.getNumberOfPoints() == 0 || m_transitioning) {
				hipTrData->m_savedPositions = LIC<3>({ hipTrans, hipTrans },{ currentFrameRepTime - ikAnim.getAnimationDuration(), currentFrameRepTime });
				hipTrData->m_motionInitialized = false;
			}
			else {
				hipTrData->m_savedPositions.insertPoint(hipTrans, currentFrameRepTime);
				// recorte de las posiciones guardadas
				for (int j = hipTrData->m_savedPositions.getNumberOfPoints() - 1; 0 <= j; j--) {
					float tVal = hipTrData->m_savedPositions.getTValue(j);
					float minVal = hipTrData->m_savedPositions.getTRange()[1] - ikAnim.getAnimationDuration() * 2;
					if (tVal < minVal) {
						hipTrData->m_savedPositions = hipTrData->m_savedPositions.sample(tVal,	hipTrData->m_savedPositions.getTRange()[1]);
						break;
					}
				}
			}

			// recalcular trayectorias de ee y caderas
			m_ikRig.calculateTrajectories(animIndex, transformManager, staticMeshManager);

			// si no hay info de posicion guardada al comenzar el movimiento, se usa la curva objetivo
			for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
				trData = ikAnim.getEETrajectoryData(i);
				if (!trData->m_motionInitialized) {
					LIC<3> targetCurve = trData->getTargetTrajectory().getEECurve();
					trData->m_savedPositions = targetCurve;
					if (!ikAnim.isMovementFixed() || ikAnim.getAnimationType()==AnimationType::IDLE) {
						trData->m_motionInitialized = true;
					}					
				}
			}
			if (!hipTrData->m_motionInitialized) {
				LIC<3> targetCurve = hipTrData->getTargetPositions();
				hipTrData->m_savedPositions = targetCurve;
				if (!ikAnim.isMovementFixed() || ikAnim.getAnimationType() == AnimationType::IDLE) {
					hipTrData->m_motionInitialized = true;
				}	
			}
			// asignar objetivos a ee's
			int repOffset_next = ikAnim.getCurrentFrameIndex() < ikAnim.getFrameNum() - 1 ? 0 : 1;
			float targetTimeNext = ikAnim.getReproductionTime(ikAnim.getNextFrameIndex(), repOffset_next);
			float targetTimeCurr = ikAnim.getReproductionTime(ikAnim.getCurrentFrameIndex());
			float deltaT = targetTimeNext - targetTimeCurr;

			glm::mat4 nextGlblTransform = glmUtils::translationToMat4(hipTrData->getTargetPositions().evalCurve(targetTimeNext)) *
				glmUtils::rotationToMat4(glm::angleAxis(m_ikRig.m_rotationAngle + m_ikRig.m_angularSpeed*deltaT, m_ikRig.getUpVector())) *
				glmUtils::scaleToMat4(glm::vec3(m_ikRig.m_rigScale));
			glm::mat4 toModelSpace = glm::inverse(nextGlblTransform);
			for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
				IKChain* ikChain = m_ikRig.getIKChain(i);
				trData = ikAnim.getEETrajectoryData(i);
				glm::vec3 eeTarget = toModelSpace *glm::vec4(trData->getTargetTrajectory().getEECurve().evalCurve(targetTimeNext), 1);
				ikChain->setCurrentEETarget(eeTarget);
			}
		}
		
		
	}

	void IKRigController::updateGlobalTransform(ComponentManager<TransformComponent>& transformManager) {
		// setear transformacion global (traslacion y direccion de movimiento)
		glm::fquat updatedRotation = glm::angleAxis(m_ikRig.m_rotationAngle, m_ikRig.getUpVector());
		transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->SetRotation(updatedRotation);
		
		glm::vec3 newGlobalPosition(0);
		int activeConfigs = 0;
		for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
			IKAnimation& ikAnim = m_ikRig.m_ikAnimations[i];
			if (ikAnim.isActive()) {
				HipGlobalTrajectoryData* hipTrData = ikAnim.getHipTrajectoryData();
				if (hipTrData->getTargetPositions().inTRange(ikAnim.getCurrentReproductionTime())) {
					newGlobalPosition += hipTrData->getTargetPositions().evalCurve(ikAnim.getCurrentReproductionTime());
					activeConfigs += 1;
				}
			}
		}
		if (0 < activeConfigs) {
			newGlobalPosition /= activeConfigs;
			transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->SetTranslation(newGlobalPosition);
		}
	}


	void IKRigController::updateAnimation(AnimationIndex animIndex) {
		IKAnimation& ikAnim = m_ikRig.m_ikAnimations[animIndex];
		if (ikAnim.m_onNewFrame) {
			FrameIndex currentFrame = ikAnim.getCurrentFrameIndex();
			FrameIndex nextFrame = ikAnim.getNextFrameIndex();
			float currentFrameRepTime = ikAnim.getReproductionTime(currentFrame);

			int repCountOffset = currentFrame < nextFrame ? 0 : 1;
			float nextFrameRepTime = ikAnim.getReproductionTime(nextFrame, repCountOffset);

			float avgFrameDuration = ikAnim.getAnimationDuration() / ikAnim.getFrameNum();

			for (int i = 0; i < m_ikRig.getChainNum(); i++) {
				for (int j = 0; j < m_ikRig.getIKChain(i)->getJoints().size() - 1; j++) {
					JointIndex jIndex = m_ikRig.getIKChain(i)->getJoints()[j];
					// recorte de los angulos guardados
					for (int k = ikAnim.m_savedAngles[jIndex].getNumberOfPoints() - 1; 0 <= k; k--) {
						float tVal = ikAnim.m_savedAngles[jIndex].getTValue(k);
						float minVal = ikAnim.m_savedAngles[jIndex].getTRange()[1] - avgFrameDuration*6;
						if (tVal < minVal) {
							ikAnim.m_savedAngles[jIndex] = ikAnim.m_savedAngles[jIndex].sample(tVal, ikAnim.m_savedAngles[jIndex].getTRange()[1]);
							break;
						}
					}
				}
			}

			// calcular nuevas rotaciones para la animacion con ik
			std::vector<std::pair<JointIndex, float>> calculatedAngles = m_ikRig.calculateRotationAngles(animIndex);
			std::shared_ptr<AnimationClip> animClip = ikAnim.m_animationClip;
			for (int i = 0; i < calculatedAngles.size(); i++) {
				JointIndex jIndex = calculatedAngles[i].first;
				float calcAngle = calculatedAngles[i].second;
				// guardamos valor calculado
				ikAnim.m_savedAngles[jIndex].insertPoint(glm::vec1(calcAngle), nextFrameRepTime);
				// actualizamos current y next frame
				float currentFrameAngle = ikAnim.getSavedAngles(jIndex).evalCurve(currentFrameRepTime)[0];
				glm::vec3 currentFrameAxis = ikAnim.m_originalJointRotations[currentFrame][jIndex].getRotationAxis();
				animClip->SetRotation(glm::angleAxis(currentFrameAngle, currentFrameAxis), currentFrame, jIndex);

				float nextFrameAngle = ikAnim.getSavedAngles(jIndex).evalCurve(nextFrameRepTime)[0];
				glm::vec3 nextFrameAxis = ikAnim.m_originalJointRotations[nextFrame][jIndex].getRotationAxis();
				animClip->SetRotation(glm::angleAxis(nextFrameAngle, nextFrameAxis), nextFrame, jIndex);
			}
			
		}
		
	}

	void IKRigController::updateIKRigConfigTime(float animationTimeStep, AnimationIndex animIndex, AnimationController& animController) {
		IKAnimation& ikAnim = m_ikRig.m_ikAnimations[animIndex];
		float avgFrameDuration = ikAnim.getAnimationDuration() / ikAnim.getFrameNum();
		std::shared_ptr<AnimationClip> animClip = ikAnim.m_animationClip;
		float prevSamplingTime = animClip->GetSamplingTime(m_reproductionTime - animationTimeStep, true);
		float samplingTimeOffset = 0.0f;
		if (animController.m_animationClipPtr == animClip) {
			samplingTimeOffset = animController.m_sampleTime - prevSamplingTime;
		}
		else if (animController.m_crossfadeTarget.GetAnimationClip() == animClip) {
			samplingTimeOffset = animController.m_crossfadeTarget.m_sampleTime - prevSamplingTime;
		}
		if ( samplingTimeOffset + avgFrameDuration / 10.0f < 0) {
			samplingTimeOffset += ikAnim.getAnimationDuration();
		}
		
		float adjustedReproductionTime = m_reproductionTime + samplingTimeOffset;
		ikAnim.m_currentReproductionTime = adjustedReproductionTime;
		ikAnim.m_reproductionCount = ikAnim.m_currentReproductionTime / ikAnim.getAnimationDuration();
		float adjustedSamplingTime = animClip->GetSamplingTime(ikAnim.m_currentReproductionTime, true);
		for (int i = 0; i < ikAnim.getTimeStamps().size(); i++) {
			float nextTimeStamp = i < ikAnim.getTimeStamps().size()-1 ? ikAnim.getTimeStamps()[i + 1] : ikAnim.getAnimationDuration();
			if (ikAnim.getTimeStamps()[i] <= adjustedSamplingTime && adjustedSamplingTime < nextTimeStamp) {
				ikAnim.m_onNewFrame = ikAnim.m_currentFrameIndex != i;
				ikAnim.m_currentFrameIndex = i;
				break;
			}
		}		
	}

	void IKRigController::refreshConfig(AnimationIndex animIndex) {
		IKAnimation& ikAnim = m_ikRig.m_ikAnimations[animIndex];
		ikAnim.refresh();
		m_ikRig.resetAnimation(animIndex);
	}

	void IKRigController::updateIKRig(float timeStep, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager, ComponentManager<SkeletalMeshComponent>& skeletalMeshManager) {
		validateTerrains(staticMeshManager);
		AnimationController& animController = skeletalMeshManager.GetComponentPointer(m_skeletalMeshHandle)->GetAnimationController();
		float animTimeStep = timeStep * animController.GetPlayRate();
		m_reproductionTime += animTimeStep;
		for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
			updateIKRigConfigTime(animTimeStep, i, animController);
		}
		updateMovementDirection(animTimeStep);
		int activeAnimations = 0;
		for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
			IKAnimation& ikAnim = m_ikRig.m_ikAnimations[i];
			if (animController.m_animationClipPtr == ikAnim.m_animationClip ||
				animController.m_crossfadeTarget.GetAnimationClip() == ikAnim.m_animationClip) {
				ikAnim.m_active = true;
				activeAnimations += 1;
			}
			else {
				ikAnim.m_active = false;
				refreshConfig(i);
			}
		}
		m_transitioning = activeAnimations == 2;
		for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
			if (m_ikRig.m_ikAnimations[i].isActive()) {
				updateTrajectories(i, transformManager, staticMeshManager);			
			}
		}
		updateGlobalTransform(transformManager);
		if (m_ikEnabled) {
			for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
				IKAnimation& ikAnim = m_ikRig.m_ikAnimations[i];
				if (ikAnim.isActive()) {
					if (ikAnim.getAnimationType() == AnimationType::WALKING) {
						if (!ikAnim.isMovementFixed()) {
							updateAnimation(i);
						}
					}
					else if (ikAnim.getAnimationType() == AnimationType::IDLE) {
						updateAnimation(i);
					}
				}
			}
		}		

		for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
			IKAnimation& ikAnim = m_ikRig.m_ikAnimations[i];
			ikAnim.m_onNewFrame = false;
		}

	}

	void IKRigController::enableIK(bool enableIK) {
		if (!enableIK) {
			for (AnimationIndex i = 0; i < m_ikRig.m_ikAnimations.size(); i++) {
				m_ikRig.resetAnimation(i);
			}
		}
		m_ikEnabled = enableIK;		
	}





}