#include "IKRigController.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Core/GlmUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"

namespace Mona {

	

	IKRigController::IKRigController(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle,
		InnerComponentHandle skeletalMeshHandle,  glm::mat4 baseGlobalTransform):
		m_skeletalMeshHandle(skeletalMeshHandle), m_ikRig(skeleton, rigData, transformHandle) {
		glm::vec3 glblScale; glm::quat glblRotation; glm::vec3 glblTranslation;	glm::vec3 glblSkew;	glm::vec4 glblPerspective;
		glm::decompose(baseGlobalTransform, glblScale, glblRotation, glblTranslation, glblSkew, glblPerspective);
		MONA_ASSERT(glmUtils::isApproxUniform(glblScale), "Global scale must be uniform");
		m_baseGlobalTransform = baseGlobalTransform;
		m_ikRig.m_rigScale = glblScale[0];
	}
	void IKRigController::init() {
		m_ikRig.init();
	}

	void IKRigController::validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager) {
		m_ikRig.m_trajectoryGenerator.m_environmentData.validateTerrains(staticMeshManager);
	}

	void IKRigController::addAnimation(std::shared_ptr<AnimationClip> animationClip) {
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
			JointIndex eeIndex = m_ikRig.m_ikChains[i].getJoints().back();
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
				}
				currentTimeIndexes[i] += 1;
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
		currentConfig->m_eeTrajectoryData = std::vector<EEGlobalTrajectoryData>(m_ikRig.m_ikChains.size());

		int chainNum = m_ikRig.m_ikChains.size();

		// numero de rotaciones por joint con la animaciond descomprimida
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();

		// minima distancia entre posiciones de un frame a otro para considerarlo un movimiento
		float minDistance = m_ikRig.m_rigHeight / 1000;

		// Guardamos la informacion de traslacion y rotacion de la cadera, antes de eliminarla
		auto hipTrack = animationClip->m_animationTracks[animationClip->GetTrackIndex(m_ikRig.m_hipJoint)];
		std::vector<float> hipTimeStamps;
		hipTimeStamps.reserve(frameNum);
		std::vector<glm::vec3> hipRotAxes;
		hipRotAxes.reserve(frameNum);
		std::vector<glm::vec1> hipRotAngles;
		hipRotAngles.reserve(frameNum);
		std::vector<glm::vec3> hipTranslations;
		hipTranslations.reserve(frameNum);
		JointIndex hipIndex = m_ikRig.m_hipJoint;
		glm::vec3 previousHipPosition(std::numeric_limits<float>::min());
		glm::vec3 hipScale; glm::quat hipRotation; glm::vec3 hipTranslation; glm::vec3 hipSkew; glm::vec4 hipPerspective;
		for (int i = 0; i < frameNum; i++) {
			float timeStamp = hipTrack.rotationTimeStamps[i];
			JointIndex parent = hipIndex;
			glm::mat4 hipTransform = glm::identity<glm::mat4>();
			while (parent != -1) {
				hipTransform *= glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, parent, true)) *
					glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, parent, true)) *
					glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, parent, true));
				parent = m_ikRig.getTopology()[parent];
			}
			glm::vec3 hipPosition = hipTransform * glm::vec4(0,0,0,1);
			if (minDistance <= glm::distance(hipPosition, previousHipPosition) || i==(frameNum-1)) { // el valor del ultimo frame se guarda si o si
				hipTransform = m_baseGlobalTransform * hipTransform; // queremos guardar valores globales
				glm::decompose(hipTransform, hipScale, hipRotation, hipTranslation, hipSkew, hipPerspective);
				hipRotAngles.push_back(glm::vec1(glm::angle(hipRotation)));
				hipRotAxes.push_back(glm::axis(hipRotation));
				hipTranslations.push_back(hipTranslation);
				hipTimeStamps.push_back(timeStamp);
			}
			previousHipPosition = hipPosition;
		}

		currentConfig->m_hipTrajectoryData.init(frameNum, currentConfig->getAnimationDuration());
		currentConfig->m_hipTrajectoryData.m_originalRotationAngles = LIC<1>(hipRotAngles, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalRotationAxes = LIC<3>(hipRotAxes, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalTranslations = LIC<3>(hipTranslations, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalFrontVector = glm::normalize(glm::vec2(hipTrack.positions.back()) - 
			glm::vec2(hipTrack.positions[0]));

		// Ahora guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		std::vector<float> rotTimeStamps = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<bool>> supportFramesPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<glm::vec3>> glblPositionsPerChain(m_ikRig.m_ikChains.size());
		std::vector<glm::vec3> positions(m_ikRig.getTopology().size());
		std::vector<glm::mat4> transforms(m_ikRig.getTopology().size());
		for (int i = 0; i < m_ikRig.getTopology().size(); i++) { transforms[i] = glm::identity<glm::mat4>(); }
		std::vector<glm::vec3> previousPositions(m_ikRig.getTopology().size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::min()));
		for (int i = 0; i < chainNum; i++) {
			supportFramesPerChain[i] = std::vector<bool>(frameNum);
			glblPositionsPerChain[i] = std::vector<glm::vec3>(frameNum);
			currentConfig->m_eeTrajectoryData[i].init(frameNum);
		}
		for (int i = 0; i < frameNum; i++) {
			// calculo de las transformaciones en model space
			float timeStamp = rotTimeStamps[i];
			for (int j = 0; j < currentConfig->getJointIndices().size(); j++) {
				JointIndex jIndex = currentConfig->getJointIndices()[j];
				glm::mat4 baseTransform = j == 0 ? glm::identity<glm::mat4>() : transforms[m_ikRig.getTopology()[jIndex]];
				transforms[jIndex] = baseTransform *
					glmUtils::translationToMat4(animationClip->GetPosition(timeStamp, jIndex, true)) *
					glmUtils::rotationToMat4(animationClip->GetRotation(timeStamp, jIndex, true)) *
					glmUtils::scaleToMat4(animationClip->GetScale(timeStamp, jIndex, true));
			}

			// calculo de las posiciones
			for (int j = 0; j < positions.size(); j++) {
				positions[j] = transforms[j] * glm::vec4(0, 0, 0, 1);
			}
			for (int j = 0; j < chainNum; j++) {
				int eeIndex = m_ikRig.m_ikChains[j].getJoints().back();
				bool isSupportFrame = glm::distance(positions[eeIndex], previousPositions[eeIndex]) <= minDistance*3;
				supportFramesPerChain[j][i] = isSupportFrame;
				glblPositionsPerChain[j][i] = m_baseGlobalTransform * glm::vec4(positions[eeIndex], 1);
			}
			previousPositions = positions;
		}

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

		// dividimos cada trayectoria global (por ee) en sub trayectorias dinamicas y estaticas.
		std::vector<bool> brokenTrajectories(chainNum);
		std::vector<bool> continueTrajectory(chainNum);
		for (int i = 0; i < chainNum; i++) {
			brokenTrajectories[i] = false;
			continueTrajectory[i] = false;
		}
		for (int i = 0; i < chainNum; i++) {
			std::vector<EETrajectory> subTrajectories;
			bool allStatic = funcUtils::conditionArray_AND(supportFramesPerChain[i]);
			if (allStatic) {
				glm::vec3 staticPos = glblPositionsPerChain[i][0];
				LIC<3> staticTr({ staticPos, staticPos }, { currentConfig->getAnimationTime(0), currentConfig->getAnimationTime(frameNum-1) });
				subTrajectories.push_back(EETrajectory(staticTr, TrajectoryType::STATIC));
				for (int j = 0; j < frameNum; j++) { currentConfig->m_eeTrajectoryData[i].m_supportHeights[j] = glblPositionsPerChain[i][j][2]; }
			}
			else {
				// encontrar primer punto de interes (dinamica luego de uno estatico)
				FrameIndex curveStartFrame = -1;
				for (int j = 1; j < frameNum; j++) {
					if (supportFramesPerChain[i][j - 1] && !supportFramesPerChain[i][j]) {
						curveStartFrame = j;
						break;
					}
				}
				MONA_ASSERT(curveStartFrame != -1, "IKRigController: There must be at least one support frame per ee trajectory.");
				float supportHeight;
				int connectedIndex = -1;
				int j = curveStartFrame;
				FrameIndex currFrame;
				while (j < frameNum + curveStartFrame) {
					currFrame = j % frameNum;
					FrameIndex initialFrame = currFrame - 1;
					bool baseFrameType = supportFramesPerChain[i][currFrame];
					TrajectoryType trType = baseFrameType ? TrajectoryType::STATIC : TrajectoryType::DYNAMIC;
					supportHeight = glblPositionsPerChain[i][currFrame][2];
					std::vector<glm::vec3> curvePoints_1 = { glblPositionsPerChain[i][initialFrame] };
					std::vector<float> tValues_1 = { currentConfig->getAnimationTime(initialFrame) };
					std::vector<glm::vec3> curvePoints_2;
					std::vector<float> tValues_2;
					std::vector<glm::vec3>* selectedCPArr = &curvePoints_1;
					std::vector<float>* selectedTVArr = &tValues_1;
					GATHER_POINTS:
					while (baseFrameType == supportFramesPerChain[i][currFrame]) {
						currentConfig->m_eeTrajectoryData[i].m_supportHeights[currFrame] = baseFrameType ? 
							glblPositionsPerChain[i][currFrame][2] : supportHeight;
						(*selectedCPArr).push_back(glblPositionsPerChain[i][currFrame]);
						(*selectedTVArr).push_back(currentConfig->getAnimationTime(currFrame));
						j++;
						currFrame = j % frameNum;
						if (j == frameNum) {
							brokenTrajectories[i] = true;
							continueTrajectory[i] = true;
							break;
						}
						if (currFrame == curveStartFrame) { break; }
						
					}
					if (continueTrajectory[i]) {
						selectedCPArr = &curvePoints_2;
						selectedTVArr = &tValues_2;
						continueTrajectory[i] = false;
						goto GATHER_POINTS;
					}
					// si es estatica la reducimos
					if (trType == TrajectoryType::STATIC) {
						if (2 <= curvePoints_1.size()) {
							curvePoints_1 = { curvePoints_1[0], curvePoints_1.back() };
							tValues_1 = { tValues_1[0], tValues_1.back() };
						}
						if (2 <= curvePoints_2.size()) {
							curvePoints_2 = { curvePoints_2[0], curvePoints_2.back() };
							tValues_2 = { tValues_2[0], tValues_2.back() };
						}
					}
					LIC<3> fullCurve;
					if (curvePoints_2.size() == 0) {
						fullCurve = LIC<3>(curvePoints_1, tValues_1);
					}
					else if (curvePoints_2.size() == 1) {
						connectedIndex = subTrajectories.size();
						LIC<3> part1(curvePoints_1, tValues_1);
						fullCurve = LIC<3>::connectPoint(part1, curvePoints_2[0], tValues_2[0], currentConfig->getAnimationDuration());
					}
					else if(1 < curvePoints_2.size()) {
						connectedIndex = subTrajectories.size();
						LIC<3> part1(curvePoints_1, tValues_1);
						LIC<3> part2(curvePoints_2, tValues_2);
						fullCurve = LIC<3>::connect(part1, part2, currentConfig->getAnimationDuration());
					}
					subTrajectories.push_back(EETrajectory(fullCurve, trType));
									
				}
				if (connectedIndex != -1) {
					// falta agregar la misma curva pero al comienzo del arreglo (con otro desplazamiento temporal)
					LIC<3> connectedCurve = subTrajectories[connectedIndex].getEECurve();
					TrajectoryType connectedTrType = subTrajectories[connectedIndex].isDynamic() ? TrajectoryType::DYNAMIC : TrajectoryType::STATIC;
					connectedCurve.offsetTValues(-currentConfig->getAnimationDuration());
					subTrajectories.push_back(EETrajectory(connectedCurve, connectedTrType));
				}
				// nos aseguramos de que las curvas esten bien ordenadas
				std::vector<EETrajectory> tempTr = subTrajectories;
				subTrajectories = {};
				while (0 < tempTr.size()) {
					float minT = std::numeric_limits<float>::max();
					int minIndex = -1;
					for (int t = 0; t < tempTr.size(); t++) {
						float currT = tempTr[t].getEECurve().getTRange()[0];
						if (currT < minT) {
							minT = currT;
							minIndex = t;
						}
					}
					subTrajectories.push_back(tempTr[minIndex]);
					tempTr.erase(tempTr.begin() + minIndex);
				}
			}			
			// asignamos las sub trayectorias a la cadena correspondiente
			currentConfig->m_eeTrajectoryData[i].m_originalSubTrajectories = subTrajectories;
		}

		// guardamos los tiempos de maxima altitud de la cadera
		LIC<3>& hipTr = currentConfig->getHipTrajectoryData()->m_originalTranslations;
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			EEGlobalTrajectoryData& trData = currentConfig->m_eeTrajectoryData[i];
			for (int j = 0; j < trData.m_originalSubTrajectories.size(); j++) {
				float maxZ = std::numeric_limits<float>::min();
				int savedIndex = -1;
				LIC<3>& currentCurve = trData.m_originalSubTrajectories[j].getEECurve();
				for (int k = 0; k < currentCurve.getNumberOfPoints(); k++) {
					float tValue = currentCurve.getTValue(k);
					if (brokenTrajectories[i]) {
						if (j == 0) {
							if (!hipTr.inTRange(tValue)) {
								tValue += currentConfig->getAnimationDuration();
							}
						}
						else if (j == trData.m_originalSubTrajectories.size() - 1) {
							if (!hipTr.inTRange(tValue)) {
								tValue -= currentConfig->getAnimationDuration();
							}
						}
					}
					float hipZ = hipTr.evalCurve(tValue)[2];
					if (maxZ < hipZ) { 
						maxZ = hipZ;
						savedIndex = k;
					}
				}
				currentConfig->m_eeTrajectoryData[i].m_originalSubTrajectories[j].m_hipMaxAltitudeIndex = savedIndex;
			}	
		}
		std::cout << glmUtils::stdVectorToString(hipTranslations);

		// Se remueve el movimiento de las caderas
		animationClip->RemoveJointRotation(m_ikRig.m_hipJoint);
		animationClip->RemoveJointTranslation(m_ikRig.m_hipJoint);
		for (FrameIndex i = 0; i < currentConfig->m_baseJointRotations.size(); i++) {
			currentConfig->m_baseJointRotations[i][m_ikRig.m_hipJoint] = JointRotation(glm::identity<glm::fquat>());
			currentConfig->m_dynamicJointRotations[i][m_ikRig.m_hipJoint] = JointRotation(glm::identity<glm::fquat>());
		}

		m_ikRig.m_currentAnim = 0;
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

	void IKRigController::updateFrontVector(float timeStep) {
		m_ikRig.m_rotationAngle += m_ikRig.m_angularSpeed * timeStep;
		m_ikRig.m_frontVector = glm::rotate(m_ikRig.m_frontVector, m_ikRig.m_rotationAngle);
	}

	void IKRigController::updateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		HipGlobalTrajectoryData* hipTrData = config.getHipTrajectoryData();
		if (config.m_onFrame) { // se realiza al llegar a un frame de la animacion
			// guardado de posiciones globales ee y cadera
			glm::mat4 baseTransform = transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->GetModelMatrix();
			FrameIndex currentFrame = config.getCurrentFrameIndex();
			std::vector<glm::mat4> globalTransforms = config.getCustomSpaceTransforms(baseTransform, currentFrame, true);
			std::vector<ChainIndex> ikChains = m_ikRig.m_trajectoryGenerator.getIKChains();

			EEGlobalTrajectoryData* trData;
			for (int i = 0; i < ikChains.size(); i++) {
				trData = config.getEETrajectoryData(ikChains[i]);
				JointIndex ee = m_ikRig.m_ikChains[ikChains[i]].getJoints().back();
				trData->m_savedPositions[currentFrame] = globalTransforms[ee] * glm::vec4(0, 0, 0, 1);
			}
			glm::mat4 hipTransform = globalTransforms[m_ikRig.m_hipJoint];
			glm::vec3 hipScale; glm::fquat hipRot; glm::vec3 hipTrans; glm::vec3 hipSkew; glm::vec4 hipPers;
			glm::decompose(hipTransform, hipScale, hipRot, hipTrans, hipSkew, hipPers);
			hipTrData->m_savedTranslations[currentFrame] = hipTrans;
			hipTrData->m_savedRotationAngles[currentFrame] = glm::angle(hipRot);
			hipTrData->m_savedRotationAxes[currentFrame] = glm::axis(hipRot);

			// recalcular trayectorias de ee y caderas
			m_ikRig.calculateTrajectories(animIndex, transformManager, staticMeshManager);

			// asignar objetivos a ee's
			float targetTime = config.getReproductionTime(config.getNextFrameIndex());
			std::vector<ChainIndex> tgChainIndices = m_ikRig.m_trajectoryGenerator.getIKChains();
			glm::mat4 toModelSpace = glm::inverse(glmUtils::translationToMat4(hipTrData->getTargetTranslation(targetTime)) *
				glmUtils::rotationToMat4(hipTrData->getTargetRotation(targetTime)) *
				glmUtils::scaleToMat4(glm::vec3(m_ikRig.m_rigScale)));
			for (int i = 0; i < tgChainIndices.size();i++) {
				ChainIndex cIndex = tgChainIndices[i];
				IKChain* ikChain = m_ikRig.getIKChain(cIndex);
				trData = config.getEETrajectoryData(cIndex);
				glm::vec3 eeTarget = toModelSpace *
					glm::vec4(trData->getTargetTrajectory().getEECurve().evalCurve(targetTime), 1);
				ikChain->setCurrentEETarget(eeTarget);
			}
		}	
		// setear transformacion global
		glm::fquat glblRot = hipTrData->getTargetRotation(config.getCurrentReproductionTime());
		glm::vec3 glblTr = hipTrData->getTargetTranslation(config.getCurrentReproductionTime());
		transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->SetRotation(glblRot);
		transformManager.GetComponentPointer(m_ikRig.getTransformHandle())->SetTranslation(glblTr);
	}
	void IKRigController::updateAnimation(AnimationIndex animIndex) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		if (config.m_onFrame) {
			// calcular nuevas rotaciones para la animacion con ik
			std::vector<std::pair<JointIndex, glm::fquat>> calculatedRotations = m_ikRig.calculateRotations(animIndex);
			auto anim = config.m_animationClip;
			FrameIndex nextFrame = config.getNextFrameIndex();
			for (int i = 0; i < calculatedRotations.size(); i++) {
				anim->SetRotation(calculatedRotations[i].second, nextFrame, calculatedRotations[i].first);
			}
		}		
	}

	void IKRigController::updateIKRigConfigTime(float time, AnimationIndex animIndex) {
		IKRigConfig& config = m_ikRig.m_animationConfigs[animIndex];
		auto anim = config.m_animationClip;
		float samplingTime = anim->GetSamplingTime(time, true);
		config.m_currentReproductionTime = config.getReproductionTime(samplingTime);
		FrameIndex savedCurrentFrameVal = config.m_currentFrameIndex;
		for (int i = 0; i < config.m_timeStamps.size(); i++) {
			float nextTimeStamp = i < config.m_timeStamps.size() ? config.m_timeStamps[i + 1] : config.getAnimationDuration();
			if (config.m_timeStamps[i] <= samplingTime && samplingTime < nextTimeStamp) {
				config.m_currentFrameIndex = i;
				config.m_nextFrameIndex = (i + 1) % (config.m_timeStamps.size());
				config.m_onFrame = config.m_currentFrameIndex != savedCurrentFrameVal;
				break;
			}
		}
		if (config.m_onFrame) {			
			// si empezamos una nueva vuelta a la animacion
			if (savedCurrentFrameVal == (config.m_timeStamps.size()-1) && config.m_currentFrameIndex == 0) {
				config.m_reproductionCount += 1;
			}
		}
	}

	void IKRigController::updateIKRig(float timeStep, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager, ComponentManager<SkeletalMeshComponent>& skeletalMeshManager) {
		validateTerrains(staticMeshManager);
		m_time += timeStep*skeletalMeshManager.GetComponentPointer(m_skeletalMeshHandle)->GetAnimationController().GetPlayRate();
		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			updateIKRigConfigTime(m_time, i);
		}
		updateFrontVector(timeStep);
		if (m_ikRig.m_currentAnim != -1) {
			updateTrajectories(m_ikRig.m_currentAnim, transformManager, staticMeshManager);
			updateAnimation(m_ikRig.m_currentAnim);
		}

		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			IKRigConfig& config = m_ikRig.m_animationConfigs[i];
			config.m_onFrame = false;
		}

	}









}