#include "IKRigController.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Core/GlmUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"

namespace Mona {

	

	IKRigController::IKRigController(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle,
		InnerComponentHandle skeletalMeshHandle, ComponentManager<TransformComponent>* transformManager):
		m_skeletalMeshHandle(skeletalMeshHandle), m_ikRig(skeleton, rigData, transformHandle) {
		glm::mat4 baseGlobalTransform = transformManager->GetComponentPointer(transformHandle)->GetModelMatrix();
		glm::vec3 glblScale; glm::quat glblRotation; glm::vec3 glblTranslation;	glm::vec3 glblSkew;	glm::vec4 glblPerspective;
		glm::decompose(baseGlobalTransform, glblScale, glblRotation, glblTranslation, glblSkew, glblPerspective);
		MONA_ASSERT(glmUtils::isApproxUniform(glblScale), "Global scale must be uniform");
		m_baseGlobalTransform = baseGlobalTransform;
		m_rigScale = glblScale;
		// descartamos la rotacion de la transformacion base
		transformManager->GetComponentPointer(transformHandle)->SetRotation(glm::identity<glm::fquat>());
	}
	void IKRigController::init() {
		m_ikRig.init(m_rigScale[0]);
	}

	void IKRigController::validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager) {
		m_ikRig.m_trajectoryGenerator.m_environmentData.validateTerrains(staticMeshManager);
	}

	void IKRigController::addAnimation(std::shared_ptr<AnimationClip> animationClip) {
		// la animacion debe tener globalmente vector front={0,1,0} y up={0,0,1}
		if (animationClip->GetSkeleton() != m_ikRig.m_skeleton) {
			MONA_LOG_ERROR("IKRigController: Input animation does not correspond to base skeleton.");
			return;
		}
		for (int i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			if (m_ikRig.m_animationConfigs[i].m_animationClip->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRigController: Animation {0} for model {1} had already been added",
					animationClip->GetAnimationName(), m_ikRig.m_skeleton->GetModelName());
				return;
			}
		}
		if (animationClip->GetTrackIndex(m_ikRig.m_hipJoint) == -1) {
			MONA_LOG_ERROR("IKRigController: Hip joint must be present in input animationClip.");
		}
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			JointIndex eeIndex = m_ikRig.m_ikChains[i].getEndEffector();
			if (animationClip->GetTrackIndex(eeIndex) == -1) {
				MONA_LOG_ERROR("IKRigController: Set end effector {0} must be present in input animationClip.",
					m_ikRig.getJointNames()[eeIndex]);
			}
		}
		
		// Chequar si los escalamientos y traslaciones son constantes por joint.
		// La cadera si puede tener traslaciones variables
		for (int i = 0; i < animationClip->m_animationTracks.size(); i++) {
			auto track = animationClip->m_animationTracks[i];
			JointIndex jIndex = animationClip->m_trackJointIndices[i];
			glm::vec3 basePosition = track.positions[0];
			glm::vec3 baseScale = track.scales[0];
			for (int j = 1; j < track.positions.size(); j++) {
				if (!glmUtils::areApproxEqual(track.positions[j], basePosition) && jIndex!=m_ikRig.m_hipJoint) {
					MONA_LOG_ERROR("IKRigController: Animation must have fixed translations per joint.");
					return;
				}
			}
			for (int j = 1; j < track.scales.size(); j++) {
				if (!glmUtils::areApproxEqual(track.scales[j], baseScale)) {
					MONA_LOG_ERROR("IKRigController: Animation must have fixed scales per joint.");
					return;
				}
			}
		}

		JointIndex parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		while (parent != -1) {
			int trackIndex = animationClip->GetTrackIndex(parent);
			auto track = animationClip->m_animationTracks[trackIndex];
			for (int j = 0; j < track.rotations.size(); j++) {
				if (track.rotations[j] != glm::identity<glm::fquat>()) {
					MONA_LOG_ERROR("IKRigController: Joints above the hip in the hierarchy cannot have rotations.");
					return;
				}
			}
			for (int j = 0; j < track.positions.size(); j++) {
				if (track.positions[j] != glm::vec3(0)) {
					MONA_LOG_ERROR("IKRigController: Joints above the hip in the hierarchy cannot have translations.");
					return;
				}
			}
		}

		// Descomprimimos las rotaciones de la animacion, repitiendo valores para que todas las articulaciones 
		// tengan el mismo numero de rotaciones
		animationClip->DecompressRotations();

		// Para este paso las rotaciones deben estar descomprimidas
		AnimationIndex newIndex = m_ikRig.m_animationConfigs.size();
		m_ikRig.m_animationConfigs.push_back(IKRigConfig(animationClip, newIndex, &m_ikRig.m_forwardKinematics));
		IKRigConfig* currentConfig = m_ikRig.getAnimationConfig(newIndex);
		currentConfig->m_eeTrajectoryData = std::vector<EEGlobalTrajectoryData>(m_ikRig.m_ikChains.size());

		int chainNum = m_ikRig.m_ikChains.size();

		// numero de rotaciones por joint con la animaciond descomprimida
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();

		// minima distancia entre posiciones de un frame a otro para considerarlo un movimiento
		float minDistance = m_ikRig.m_rigHeight / 1000;


		

		// Guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		std::vector<float> rotTimeStamps = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<bool>> supportFramesPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<glm::vec3>> glblPositionsPerChain(m_ikRig.m_ikChains.size());
		std::vector<glm::vec3> glblPositions(m_ikRig.getTopology().size());
		std::vector<glm::mat4> glblTransforms(m_ikRig.getTopology().size());
		std::vector<glm::mat4> hipGlblTransforms;
		float floorZ = std::numeric_limits<float>::max(); // altura del piso para la animacion
		for (int i = 0; i < m_ikRig.getTopology().size(); i++) { glblTransforms[i] = glm::identity<glm::mat4>(); }
		std::vector<glm::vec3> previousPositions(m_ikRig.getTopology().size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::lowest()));
		for (int i = 0; i < chainNum; i++) {
			supportFramesPerChain[i] = std::vector<bool>(frameNum);
			glblPositionsPerChain[i] = std::vector<glm::vec3>(frameNum);
			currentConfig->m_eeTrajectoryData[i].init(currentConfig);
		}
		for (int i = 0; i < frameNum; i++) {
			// calculo de las transformaciones
			float timeStamp = rotTimeStamps[i];
			while (currentConfig->getAnimationDuration() <= timeStamp) { timeStamp -= 0.000001; }
			for (int j = 0; j < currentConfig->getJointIndices().size(); j++) {
				JointIndex jIndex = currentConfig->getJointIndices()[j];
				glm::mat4 baseTransform = j == 0 ? m_baseGlobalTransform : glblTransforms[m_ikRig.getTopology()[jIndex]];
				glblTransforms[jIndex] = baseTransform *
					glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, jIndex, true)) *
					glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, jIndex, true)) *
					glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, jIndex, true));
			}
			hipGlblTransforms.push_back(glblTransforms[m_ikRig.m_hipJoint]);

			// calculo de las posiciones
			for (int j = 0; j < glblPositions.size(); j++) {
				glblPositions[j] = glblTransforms[j] * glm::vec4(0, 0, 0, 1);
			}
			for (ChainIndex j = 0; j < chainNum; j++) {
				int eeIndex = m_ikRig.m_ikChains[j].getEndEffector();
				bool isSupportFrame = glm::distance(glblPositions[eeIndex], previousPositions[eeIndex]) <= minDistance*15;
				supportFramesPerChain[j][i] = isSupportFrame;
				glblPositionsPerChain[j][i] = glm::vec4(glblPositions[eeIndex], 1);
			}
			for (int j = 0; j < currentConfig->getJointIndices().size(); j++) {
				JointIndex jIndex = currentConfig->getJointIndices()[j];
				if (glblPositions[jIndex][2] < floorZ) {
					floorZ = glblPositions[jIndex][2];
				}
			}
			previousPositions = glblPositions;
		}
		// ajuste de las alturas con el suelo
		for (int i = 0; i < chainNum; i++) {
			for (int j = 0; j < frameNum; j++) {
				glblPositionsPerChain[i][j][2] -= floorZ;
			}
		}

		// Guardamos la informacion de traslacion y rotacion de la cadera, antes de eliminarla
		TrajectoryGenerator::buildHipTrajectory(currentConfig, hipGlblTransforms, minDistance, floorZ);
		
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
		TrajectoryGenerator::buildEETrajectories(currentConfig, supportFramesPerChain, glblPositionsPerChain);

		// Se remueve el movimiento de las caderas y se setea la rotacion basal
		glm::vec3 baseScale; glm::quat baseRotation; glm::vec3 baseTranslation; glm::vec3 baseSkew; glm::vec4 basePerspective;
		glm::decompose(m_baseGlobalTransform, baseScale, baseRotation, baseTranslation, baseSkew, basePerspective);
		animationClip->RemoveJointTranslation(m_ikRig.m_hipJoint);
		for (FrameIndex i = 0; i < currentConfig->getFrameNum(); i++) {
			currentConfig->m_baseJointRotations[i][m_ikRig.m_hipJoint] = JointRotation(baseRotation);
			currentConfig->m_dynamicJointRotations[i][m_ikRig.m_hipJoint] = JointRotation(baseRotation);
			animationClip->SetRotation(baseRotation, i, m_ikRig.m_hipJoint);
		}
		currentConfig->m_jointPositions[m_ikRig.m_hipJoint] = glm::vec3(0);
	}

	AnimationIndex IKRigController::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		for (int i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			if (m_ikRig.m_animationConfigs[i].m_animationClip == animationClip) {
				m_ikRig.m_animationConfigs.erase(m_ikRig.m_animationConfigs.begin() + i);
				return i;
			}
		}
		return -1;
	}

	void IKRigController::updateMovementDirection(float timeStep) {
		m_ikRig.m_rotationAngle += m_ikRig.m_angularSpeed * timeStep;
	}

	float lastTrajectoryUpdateTime = 0;
	void IKRigController::updateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		HipGlobalTrajectoryData* hipTrData = config.getHipTrajectoryData();
		FrameIndex currentFrame = config.getCurrentFrameIndex();
		EEGlobalTrajectoryData* trData;
		if (config.isActive()) {
			if (config.m_onNewFrame) { // se realiza al llegar a un frame de la animacion
			// guardado de posiciones globales ee y cadera
				std::vector<JointIndex> endEffectors;
				for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
					endEffectors.push_back(m_ikRig.m_ikChains[i].getEndEffector());
				}
				glm::mat4 baseTransform = transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->GetModelMatrix();
				std::vector<glm::mat4> globalTransforms = config.getEEListCustomSpaceTransforms(endEffectors, baseTransform, currentFrame, true);
				for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
					trData = config.getEETrajectoryData(i);
					JointIndex ee = endEffectors[i];
					trData->m_savedPositions[currentFrame] = globalTransforms[ee] * glm::vec4(0, 0, 0, 1);
					trData->m_savedDataValid[currentFrame] = true;
					// para compensar el poco espacio entre en ultimo y el primer frame
					if (currentFrame == config.getFrameNum() - 2) {
						trData->m_savedDataValid[0] = true;
						trData->m_savedPositions[0] = globalTransforms[ee] * glm::vec4(0, 0, 0, 1);
					}
				}
				glm::mat4 hipTransform = globalTransforms[m_ikRig.m_hipJoint];
				glm::vec3 hipScale; glm::fquat hipRot; glm::vec3 hipTrans; glm::vec3 hipSkew; glm::vec4 hipPers;
				glm::decompose(hipTransform, hipScale, hipRot, hipTrans, hipSkew, hipPers);
				hipTrData->m_savedTranslations[currentFrame] = hipTrans;
				hipTrData->m_savedDataValid[currentFrame] = true;
				// para compensar el poco espacio entre en ultimo y el primer frame
				if (currentFrame == config.getFrameNum() - 2) {
					hipTrData->m_savedTranslations[0] = hipTrans;
					hipTrData->m_savedDataValid[0] = true;
				}

				// recalcular trayectorias de ee y caderas
				int repOffset_next = config.getCurrentFrameIndex() < config.getFrameNum() - 1 ? 0 : 1;
				bool updateNeeded = m_reproductionTime - lastTrajectoryUpdateTime > 0.05f;
				if (!updateNeeded) {
					for (int i = 0; i < config.m_eeTrajectoryData.size(); i++) {
						float nextFrameRepTime = config.getReproductionTime(config.getNextFrameIndex(), repOffset_next);
						if (!config.getEETrajectoryData(i)->getTargetTrajectory().getEECurve().inTRange(nextFrameRepTime)) {
							updateNeeded = true;
							break;
						}
					}
				}
				if (updateNeeded) {
					lastTrajectoryUpdateTime = m_reproductionTime;
					m_ikRig.calculateTrajectories(animIndex, transformManager, staticMeshManager);
				}

				// asignar objetivos a ee's
				float targetTimeNext = config.getReproductionTime(config.getNextFrameIndex(), repOffset_next);
				float targetTimeCurr = config.getReproductionTime(config.getCurrentFrameIndex());
				float deltaT = targetTimeNext - targetTimeCurr;
				// DEBUG
				std::vector<glm::vec3> nextFrameModelSpacePos =  m_ikRig.m_forwardKinematics.ModelSpacePositions(animIndex, config.m_nextFrameIndex, true);
				std::cout << "next model space pos: next frame "<< config.m_nextFrameIndex << std::endl;
				glmUtils::printColoredStdVector(nextFrameModelSpacePos);
				// DEBUG

				glm::mat4 nextGlblTransform = glmUtils::translationToMat4(hipTrData->getTargetTranslation(targetTimeNext)) *
					glmUtils::rotationToMat4(glm::angleAxis(m_ikRig.m_rotationAngle + m_ikRig.m_angularSpeed*deltaT, m_ikRig.getUpVector())) *
					glmUtils::scaleToMat4(m_rigScale);
				glm::mat4 toModelSpace = glm::inverse(nextGlblTransform);
				for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
					IKChain* ikChain = m_ikRig.getIKChain(i);
					trData = config.getEETrajectoryData(i);
					glm::vec3 eeTarget = toModelSpace *
						glm::vec4(trData->getTargetTrajectory().getEECurve().evalCurve(targetTimeNext), 1);
					ikChain->setCurrentEETarget(eeTarget);
				}
				// asignar info de rotacion a la cadera en la animacion
				config.m_animationClip->SetRotation(hipTrData->getTargetRotation(targetTimeCurr),
					config.getCurrentFrameIndex(), m_ikRig.m_hipJoint);
				config.m_animationClip->SetRotation(hipTrData->getTargetRotation(targetTimeNext),
					config.getNextFrameIndex(), m_ikRig.m_hipJoint);
			}
			// setear transformacion global (traslacion y direccion de movimiento)
			// nueva rotacion
			glm::fquat updatedRotation = glm::angleAxis(m_ikRig.m_rotationAngle, m_ikRig.getUpVector());
			transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->SetRotation(updatedRotation);
			glm::vec3 glblTr = hipTrData->getTargetTranslation(config.getCurrentReproductionTime());
			transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->SetTranslation(glblTr);
		}
		else {
			if (config.m_onNewFrame) {
				for (ChainIndex i = 0; i < m_ikRig.getChainNum(); i++) {
					trData = config.getEETrajectoryData(i);
					trData->m_savedDataValid[currentFrame] = false;
					// para compensar el poco espacio entre en ultimo y el primer frame
					if (currentFrame == config.getFrameNum() - 2) {
						trData->m_savedDataValid[0] = false;
					}
				}
				hipTrData->m_savedDataValid[currentFrame] = false;
				// para compensar el poco espacio entre en ultimo y el primer frame
				if (currentFrame == config.getFrameNum() - 2) {
					hipTrData->m_savedDataValid[0] = false;
				}
			}
		}
		
	}


	FrameIndex lastUpdatedFrame = -1;
	void IKRigController::updateAnimation(AnimationIndex animIndex) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		if (config.m_onNewFrame) {
			FrameIndex currFrame = config.getCurrentFrameIndex();
			FrameIndex nextFrame = config.getNextFrameIndex();
			// calcular nuevas rotaciones para la animacion con ik
			std::vector<std::pair<JointIndex, glm::fquat>> calculatedRotations = m_ikRig.calculateRotations(animIndex, nextFrame);
			auto anim = config.m_animationClip;
			for (int i = 0; i < calculatedRotations.size(); i++) {
				anim->SetRotation(calculatedRotations[i].second, nextFrame, calculatedRotations[i].first);
				// para compensar el poco espacio entre en ultimo y el primer frame
				if (nextFrame == config.getFrameNum() - 1) {
					anim->SetRotation(calculatedRotations[i].second, 0, calculatedRotations[i].first);
				}
				// si el current frame no fue actualizado, le asignamos el valor calculado para next frame
				if (lastUpdatedFrame != currFrame) {
					anim->SetRotation(calculatedRotations[i].second, currFrame, calculatedRotations[i].first);
				}
			}
			if (nextFrame == config.getFrameNum() - 1) {
				lastUpdatedFrame = 0;
			}
			else {
				lastUpdatedFrame = nextFrame;
			}
		}
		
	}

	void IKRigController::updateIKRigConfigTime(float animationTimeStep, AnimationIndex animIndex) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		float avgFrameDuration = config.getAnimationDuration() / config.getFrameNum();
		if (avgFrameDuration*2 < animationTimeStep) {
			MONA_LOG_ERROR("IKRigController: Framerate is too low for IK system to work properly on animation with name --{0}--.",
				config.m_animationClip->GetAnimationName());
		}
		auto anim = config.m_animationClip;
		float samplingTime = anim->GetSamplingTime(m_reproductionTime, true);
		config.m_currentReproductionTime = m_reproductionTime;
		for (int i = 0; i < config.m_timeStamps.size(); i++) {
			float nextTimeStamp = i < config.m_timeStamps.size() ? config.m_timeStamps[i + 1] : config.getAnimationDuration();
			if (config.m_timeStamps[i] <= samplingTime && samplingTime < nextTimeStamp) {
				config.m_onNewFrame = config.m_currentFrameIndex != i;
				config.m_currentFrameIndex = i;
				config.m_nextFrameIndex = (i + 1) % (config.m_timeStamps.size());
				break;
			}
		}
		config.m_reproductionCount = m_reproductionTime / config.getAnimationDuration();
	}

	void IKRigController::updateIKRig(float timeStep, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager, ComponentManager<SkeletalMeshComponent>& skeletalMeshManager) {
		validateTerrains(staticMeshManager);
		float animTimeStep = timeStep * skeletalMeshManager.GetComponentPointer(m_skeletalMeshHandle)->GetAnimationController().GetPlayRate();
		m_reproductionTime += animTimeStep;
		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			updateIKRigConfigTime(animTimeStep, i);
		}
		updateMovementDirection(animTimeStep);
		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			updateTrajectories(i, transformManager, staticMeshManager);
			if (m_ikRig.m_animationConfigs[i].isActive()) {
				updateAnimation(i);
			}
		}

		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			IKRigConfig& config = m_ikRig.m_animationConfigs[i];
			config.m_onNewFrame = false;
		}

	}









}