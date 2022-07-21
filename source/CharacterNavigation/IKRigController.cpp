#include "IKRigController.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Core/GlmUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"

namespace Mona {


	IKRigController::IKRigController(InnerComponentHandle skeletalMeshHandle, IKRig ikRig): m_skeletalMeshHandle(skeletalMeshHandle), m_ikRig(ikRig) {

	}

	void IKRigController::validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager) {
		m_ikRig.m_trajectoryGenerator.m_environmentData.validateTerrains(staticMeshManager);
	}

	void IKRigController::addAnimation(std::shared_ptr<AnimationClip> animationClip) {
		if (animationClip->GetSkeleton() != m_ikRig.m_skeleton) {
			MONA_LOG_ERROR("IKRig: Input animation does not correspond to base skeleton.");
			return;
		}
		for (int i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			if (m_ikRig.m_animationConfigs[i].m_animationClip->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added",
					animationClip->GetAnimationName(), m_ikRig.m_skeleton->GetModelName());
				return;
			}
		}
		
		// Chequar si los escalamientos y traslaciones son constantes por joint.
		// La cadera si puede tener traslaciones variables
		for (JointIndex i = 0; i < animationClip->m_jointTrackIndices.size(); i++) {
			auto track = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[i]];
			glm::vec3 basePosition = track.positions[0];
			glm::vec3 baseScale = track.scales[0];
			for (int j = 1; j < track.positions.size(); j++) {
				if (track.positions[j] != basePosition && i!=m_ikRig.m_hipJoint) {
					MONA_LOG_ERROR("IKRigController: Animation must have fixed translations per joint.");
					return;
				}
			}
			for (int j = 1; j < track.scales.size(); j++) {
				if (track.scales[j] != baseScale) {
					MONA_LOG_ERROR("IKRigController: Animation must have fixed scales per joint.");
					return;
				}
			}
		}

		auto hipTrack = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[m_ikRig.m_hipJoint]];
		for (int i = 0; i < hipTrack.scales.size(); i++) {
			if (hipTrack.scales[i] != glm::vec3(hipTrack.scales[i][0])) {
				MONA_LOG_ERROR("IKRigController: Hip joint must have uniform scale.");
				return;
			}
		}

		JointIndex parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		while (parent != -1) {
			auto track = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[parent]];
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
			for (int j = 0; j < track.scales.size(); j++) {
				if (track.scales[j] != glm::vec3(track.scales[j][0])) {
					MONA_LOG_ERROR("IKRigController: Joints above the hip in the hierarchy must have uniform scales.");
					return;
				}
			}
		}

		// se eliminan las escalas de cadera hacia arriba en la jerarquia
		/*parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		while (parent != -1) {
			animationClip->RemoveJointScaling(parent);
			parent = m_ikRig.getTopology()[parent];
		}
		animationClip->RemoveJointScaling(m_ikRig.m_hipJoint);*/	

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

		// numero de rotaciones por joint con la animaciond descomprimida
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();

		// minima distancia entre posiciones de un frame a otro para considerarlo un movimiento
		float minDistance = m_ikRig.m_rigHeight / 1000;

		// Guardamos la informacion de traslacion y rotacion de la cadera, antes de eliminarla
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
			if (minDistance <= glm::distance(hipPosition, previousHipPosition) || i==(frameNum-1)) {
				glm::vec3 scale;
				glm::quat rotation;
				glm::vec3 translation;
				glm::vec3 skew;
				glm::vec4 perspective;
				glm::decompose(hipTransform, scale, rotation, translation, skew, perspective);
				hipRotAngles.push_back(glm::vec1(glm::angle(rotation)));
				hipRotAxes.push_back(glm::axis(rotation));
				hipTranslations.push_back(translation);
				hipTimeStamps.push_back(timeStamp);
			}
			previousHipPosition = hipPosition;
		}

		currentConfig->m_hipTrajectoryData.m_originalRotationAngles = LIC<1>(hipRotAngles, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalRotationAxes = LIC<3>(hipRotAxes, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalTranslations = LIC<3>(hipTranslations, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalTranslations.scale(glm::vec3(m_ikRig.m_scale));
		currentConfig->m_hipTrajectoryData.m_originalFrontVector = glm::normalize(glm::vec2(hipTrack.positions.back()) - 
			glm::vec2(hipTrack.positions[0]));

		// Ahora guardamos las trayectorias originales de los ee y definimos sus frames de soporte
		std::vector<float> rotTimeStamps = animationClip->m_animationTracks[0].rotationTimeStamps;
		std::vector<std::vector<bool>> supportFramesPerChain(m_ikRig.m_ikChains.size());
		std::vector<std::vector<glm::vec3>> glblPositionsPerChain(m_ikRig.m_ikChains.size());
		std::vector<glm::vec3> positions(m_ikRig.getTopology().size());
		std::vector<glm::mat4> transforms(m_ikRig.getTopology().size());
		std::vector<glm::vec3> previousPositions(m_ikRig.getTopology().size());
		std::fill(previousPositions.begin(), previousPositions.end(), glm::vec3(std::numeric_limits<float>::min()));
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			supportFramesPerChain[i] = std::vector<bool>(frameNum);
			glblPositionsPerChain[i] = std::vector<glm::vec3>(frameNum);
		}
		for (int i = 0; i < frameNum; i++) {
			// calculo de las transformaciones
			float timeStamp = rotTimeStamps[i];
			for (int j = 0; j < m_ikRig.getTopology().size(); j++) {
				glm::mat4 baseTransform = j == 0 ? glm::identity<glm::mat4>() : transforms[m_ikRig.getTopology()[j]];
				transforms[j] = baseTransform *
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
				supportFramesPerChain[j][i] = isSupportFrame;
				glblPositionsPerChain[j][i] = glm::vec3(m_ikRig.m_scale)*positions[eeIndex];
			}
			previousPositions = positions;
		}
		// a los valores de soporte del primer frame les asignamos el valor del ultimo asumiento circularidad
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			supportFramesPerChain[i][0] = supportFramesPerChain[i].back();
		}

		// dividimos cada trayectoria global (por ee) en sub trayectorias dinamicas y estaticas.
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			// encontrar primer punto de soporte
			FrameIndex firstSF = -1;
			for (int j = 0; j < frameNum; j++) {
				if (supportFramesPerChain[i][j]) {
					firstSF = j;
					break;
				}
			}
			MONA_ASSERT(firstSF != -1, "IKRigController: There must be at least one support frame per ee trajectory.");
			std::vector<EETrajectory> subTrajectories;
			float supportHeight;
			for (FrameIndex j = firstSF; j < frameNum; j++) {
				if (supportFramesPerChain[i][j]) {
					FrameIndex initialFrame = 0 < j ? (j - 1) : (frameNum - 1);
					FrameIndex finalFrame;
					while (supportFramesPerChain[i][j]) {
						supportHeight = glblPositionsPerChain[i][j][2];
						currentConfig->m_ikChainTrajectoryData[i].m_supportHeights[j] = supportHeight;
						finalFrame = j;
						j += 1;
						if (j == frameNum) {
							j = 0; // volvemos al principio para completar las curvas (dinamicas) faltantes
							break; 
						}
					}
					// armar trayectoria estatica
					glm::vec3 staticPos = glblPositionsPerChain[i][j];
					LIC<3> staticTr({ staticPos, staticPos }, { currentConfig->getAnimationTime(initialFrame), 
						currentConfig->getAnimationTime(finalFrame) });
					subTrajectories.push_back(EETrajectory(staticTr, TrajectoryType::STATIC));

				}
				else {
					FrameIndex initialFrame = 0 < j ? (j - 1) : (frameNum - 1);
					bool incompleteTr = false;
					std::vector<glm::vec3> curvePoints_1 = { glblPositionsPerChain[i][initialFrame]};
					std::vector<float> tValues_1 = {currentConfig->getAnimationTime(initialFrame)};
					while (!supportFramesPerChain[i][j]) {
						currentConfig->m_ikChainTrajectoryData[i].m_supportHeights[j] = supportHeight;
						curvePoints_1.push_back(glblPositionsPerChain[i][j]);
						tValues_1.push_back(currentConfig->getAnimationTime(j));
						j += 1;
						if (j == frameNum) {
							incompleteTr = true;
							break; 
						}
					}
					if (!incompleteTr) { // si llegamos a un frame de soporte sin salirnos del arreglo
						supportHeight = glblPositionsPerChain[i][j][2];
						currentConfig->m_ikChainTrajectoryData[i].m_supportHeights[j] = supportHeight;
						subTrajectories.push_back(EETrajectory(LIC<3>(curvePoints_1, tValues_1), TrajectoryType::DYNAMIC));
					}
					else {
						std::vector<glm::vec3> curvePoints_2;
						std::vector<float> tValues_2;
						for (FrameIndex k = 0; k < firstSF; k++) {
							currentConfig->m_ikChainTrajectoryData[i].m_supportHeights[k] = supportHeight;
							curvePoints_2.push_back(glblPositionsPerChain[i][k]);
							tValues_2.push_back(currentConfig->getAnimationTime(k));
						}
						LIC<3> part1(curvePoints_1, tValues_1);
						LIC<3> part2(curvePoints_2, tValues_2);
						// desplazamos las posiciones de la parte 2 para que quede pegada a la parte 1
						part2.translate(part1.getEnd() - part2.getStart());
						// luego hacemos el desplazamiento temporal
						part2.offsetTValues(currentConfig->getAnimationDuration());
						LIC<3> dynamicCurve = LIC<3>::join(part1, part2);
						subTrajectories.push_back(EETrajectory(dynamicCurve, TrajectoryType::DYNAMIC));
						// falta agregar la misma curva pero al comienzo del arreglo (con otro desplazamiento temporal)
						dynamicCurve.offsetTValues(-currentConfig->getAnimationDuration());
						subTrajectories.insert(subTrajectories.begin() ,EETrajectory(dynamicCurve, TrajectoryType::DYNAMIC));
					}
				}				
			}
			// asignamos las sub trayectorias a la cadena correspondiente
			currentConfig->m_ikChainTrajectoryData[i].m_originalSubTrajectories = subTrajectories;
		}

		// guardamos los tiempos de maxima altitud de la cadera
		LIC<3>& hipTr = currentConfig->getHipTrajectoryData()->m_originalTranslations;
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			EEGlobalTrajectoryData& trData = currentConfig->m_ikChainTrajectoryData[i];
			for (int j = 0; j < trData.m_originalSubTrajectories.size(); j++) {
				float maxZ = std::numeric_limits<float>::min();
				int savedIndex = -1;
				LIC<3>& currentCurve = trData.m_originalSubTrajectories[j].getEECurve();
				for (int k = 0; k < currentCurve.getNumberOfPoints(); k++) {
					float tValue = currentCurve.getTValue(k);
					float hipZ = hipTr.evalCurve(tValue)[2];
					if (maxZ < hipZ) { 
						maxZ = hipZ;
						savedIndex = k;
					}
				}
				currentConfig->m_ikChainTrajectoryData[i].m_originalSubTrajectories[j].m_hipMaxAltitudeIndex = savedIndex;
			}	
		}

		// Se remueve el movimiento de las caderas
		animationClip->RemoveJointRotation(m_ikRig.m_hipJoint);
		animationClip->RemoveJointTranslation(m_ikRig.m_hipJoint);
		for (int i = 0; i < currentConfig->m_baseJointRotations[m_ikRig.m_hipJoint].size(); i++) {
			currentConfig->m_baseJointRotations[m_ikRig.m_hipJoint][i] = JointRotation(glm::identity<glm::fquat>());
		}

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

			HipGlobalTrajectoryData* hipTrData = config.getHipTrajectoryData();
			glm::mat4 hipTransform = globalTransforms[m_ikRig.m_hipJoint];
			glm::vec3 hipScale; glm::fquat hipRot; glm::vec3 hipTrans; glm::vec3 hipSkew; glm::vec4 hipPers;
			glm::decompose(hipTransform, hipScale, hipRot, hipTrans, hipSkew, hipPers);
			hipTrData->m_savedTranslations[currentFrame] = hipTrans;
			hipTrData->m_savedRotationAngles[currentFrame] = glm::angle(hipRot);
			hipTrData->m_savedRotationAxes[currentFrame] = glm::axis(hipRot);

			// recalcular trayectorias de ee y caderas
			m_ikRig.calculateTrajectories(animIndex, transformManager, staticMeshManager);
		}		
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
		updateTrajectories(m_ikRig.m_currentAnim, transformManager, staticMeshManager);
		updateAnimation(m_ikRig.m_currentAnim);

		for (AnimationIndex i = 0; i < m_ikRig.m_animationConfigs.size(); i++) {
			IKRigConfig& config = m_ikRig.m_animationConfigs[i];
			config.m_onFrame = false;
		}

	}









}