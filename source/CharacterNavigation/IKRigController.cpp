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


		JointIndex parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		while (parent != -1) {
			auto track = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[0]];
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

		// se eliminan las escalas de cadera hacia arriba en la jerarquia
		parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		while (parent != -1) {
			animationClip->RemoveJointScaling(parent);
			parent = m_ikRig.getTopology()[m_ikRig.m_hipJoint];
		}
		animationClip->RemoveJointScaling(m_ikRig.m_hipJoint);	

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

		// numero de rotaciones por joint con la animaciond descomprimida
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();

		// minima distancia entre posiciones de un frame a otro para considerarlo un movimiento
		float minDistance = m_ikRig.m_rigHeight / 1000;

		// Guardamos la informacion de traslacion y rotacion de la cadera, antes de eliminarla
		auto hipTrack = animationClip->m_animationTracks[animationClip->m_jointTrackIndices[m_ikRig.m_hipJoint]];
		std::vector<float> hipTimeStamps;
		hipTimeStamps.reserve(frameNum);
		std::vector<glm::vec3> hipRotAxes;
		hipRotAxes.reserve(frameNum);
		std::vector<glm::vec1> hipRotAngles;
		hipRotAngles.reserve(frameNum);
		std::vector<glm::vec3> hipTranslations;
		hipTranslations.reserve(frameNum);
		std::vector<glm::vec3> hipPositions;
		hipPositions.reserve(frameNum);
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
			glm::vec3 hipPosition = hipTransform * glm::identity<glm::vec4>();
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
				hipPositions.push_back(hipPosition);
				hipTimeStamps.push_back(timeStamp);
			}
			previousHipPosition = hipPosition;
		}

		currentConfig->m_hipTrajectoryData.m_originalRotationAngles = LIC<1>(hipRotAngles, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalRotationAxes = LIC<3>(hipRotAxes, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalTranslations = LIC<3>(hipTranslations, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalTranslations.scale(glm::vec3(m_ikRig.m_scale));
		currentConfig->m_hipTrajectoryData.m_originalTrajectory = LIC<3>(hipPositions, hipTimeStamps);
		currentConfig->m_hipTrajectoryData.m_originalTrajectory.scale(glm::vec3(m_ikRig.m_scale));
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
					FrameIndex initialFrame = 0 < (j - 1) ? (j - 1) : (frameNum - 1);
					FrameIndex finalFrame;
					while (supportFramesPerChain[i][j]) {
						supportHeight = glblPositionsPerChain[i][j][2];
						currentConfig->m_ikChainTrajectoryData[i].m_supportHeights[j] = supportHeight;
						finalFrame = j;
						j += 1;
						if (j == frameNum) { break; }
					}
					// armar trayectoria estatica
					glm::vec3 staticPos = glblPositionsPerChain[i][j];
					LIC<3> staticTr({ staticPos, staticPos }, { currentConfig->getAnimationTime(initialFrame), 
						currentConfig->getAnimationTime(finalFrame) });
					subTrajectories.push_back(EETrajectory(staticTr, TrajectoryType::STATIC));

				}
				else {
					FrameIndex initialFrame = j - 1;
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
						LIC<3> dynamicTr = LIC<3>::join(part1, part2);
						subTrajectories.push_back(EETrajectory(dynamicTr, TrajectoryType::DYNAMIC));
						// falta agregar la misma curva pero al comienzo del arreglo (con otro desplazamiento temporal)
						dynamicTr.offsetTValues(-currentConfig->getAnimationDuration());
						subTrajectories.insert(subTrajectories.begin() ,EETrajectory(dynamicTr, TrajectoryType::DYNAMIC));
					}
				}				
			}
			// asignamos las sub trayectorias a la cadena correspondiente
			currentConfig->m_ikChainTrajectoryData[i].m_originalSubTrajectories = subTrajectories;
		}

		// guardamos los tiempos de maxima altitud de la cadera
		LIC<3>& hipTr = currentConfig->getHipTrajectoryData()->m_originalTrajectory;
		for (int i = 0; i < m_ikRig.m_ikChains.size(); i++) {
			EEGlobalTrajectoryData& trData = currentConfig->m_ikChainTrajectoryData[i];
			for (int j = 0; j < trData.m_originalSubTrajectories.size(); j++) {
				float maxZ = std::numeric_limits<float>::min();
				float savedT = std::numeric_limits<float>::min();
				LIC<3>& currentCurve = trData.m_originalSubTrajectories[j].getEETrajectory();
				for (int k = 0; k < currentCurve.getNumberOfPoints(); k++) {
					float tValue = currentCurve.getTValue(k);
					float hipZ = hipTr.evalCurve(tValue)[2];
					if (maxZ < hipZ) { 
						maxZ = hipZ;
						savedT = tValue;
					}
				}
				currentConfig->m_ikChainTrajectoryData[i].m_originalSubTrajectories[j].m_hipMaxAltitudeTimeFraction = 
					funcUtils::getFraction(currentCurve.getTRange()[0], currentCurve.getTRange()[1], savedT);
			}	
		}

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
		config.m_currentReproductionTime = config.getReproductionTime(samplingTime);
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