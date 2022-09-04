#include "TrajectoryGenerator.hpp"
#include "../Core/GlmUtils.hpp"
#include "../Core/FuncUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "IKRig.hpp"
#include "../World/ComponentManager.hpp"
#include "../Animation/AnimationClip.hpp"
#include <algorithm>

namespace Mona{


    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig) {
        m_ikRig = ikRig;
		m_strideValidationEnabled = false;
		m_strideCorrectionEnabled = true;
    }

	void TrajectoryGenerator::init() {
		m_strideCorrector.init();
	}

	void TrajectoryGenerator::generateNewTrajectories(AnimationIndex animIndex,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		// las trayectorias anteriores siempre deben llegar hasta el currentFrame

        IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);

        for (ChainIndex i = 0; i < m_ikRig->getChainNum(); i++) {
			JointIndex eeIndex = m_ikRig->getIKChain(i)->getEndEffector();
			generateEETrajectory(i, config, transformManager, staticMeshManager);
		}

		// fijamos o desfijamos la animacion si es necesario
		FrameIndex prevSavedFixedFrame = config->m_fixedMovementFrame;
		if (!config->isMovementFixed()) {
			config->m_fixedMovementFrame = -1;
			if (prevSavedFixedFrame != -1) {
				m_ikRig->resetAnimation(config->m_animIndex);
			}
		}
		else {
			m_ikRig->fixAnimation(config->m_animIndex, config->m_fixedMovementFrame);
		}
        // si una trayectoria es dinamica es fija las demas tambien debera serlo
        if (config->isMovementFixed()) {
			FrameIndex currentFrame = config->getCurrentFrameIndex();
			float currentRepTime = config->getReproductionTime(currentFrame);
            for (ChainIndex i = 0; i < m_ikRig->getChainNum(); i++) {
				EEGlobalTrajectoryData* trData = config->getEETrajectoryData(i);
				if (trData->getTargetTrajectory().isDynamic()) {
					if (!trData->isTargetFixed()) {
						glm::vec3 currentPos = trData->getSavedPosition(currentRepTime);
						int trID = trData->getTargetTrajectory().getSubTrajectoryID();
						float currSupportHeight = trData->getSupportHeight(config->m_fixedMovementFrame);
						generateFixedTrajectory(glm::vec2(currentPos), { currentRepTime, currentRepTime + config->getAnimationDuration() },
							trID, currSupportHeight, trData, transformManager, staticMeshManager);
					}
					EEGlobalTrajectoryData* oppositeTrData = trData->getOppositeTrajectoryData();
					float oppositeCurrSupportHeight = oppositeTrData->getSupportHeight(config->m_fixedMovementFrame);
					glm::vec3 oppositeCurrentPos = oppositeTrData->getSavedPosition(currentRepTime);
					int oppositeTrID = oppositeTrData->getTargetTrajectory().getSubTrajectoryID();
					generateFixedTrajectory(glm::vec2(oppositeCurrentPos), { currentRepTime, currentRepTime + config->getAnimationDuration() },
						oppositeTrID, oppositeCurrSupportHeight, oppositeTrData, transformManager, staticMeshManager);
				}
            }
        }

		// queda setear la trayectoria de la cadera
		generateHipTrajectory(config, transformManager, staticMeshManager);
	}

    void TrajectoryGenerator::generateFixedTrajectory(glm::vec2 basePos,
        glm::vec2 timeRange, int baseCurveID, float supportHeight,	EEGlobalTrajectoryData* trData,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
		IKRigConfig* config = trData->m_config;
        float calcHeight = m_environmentData.getTerrainHeight(glm::vec2(basePos), transformManager, staticMeshManager);
        glm::vec3 fixedPos(basePos, calcHeight + supportHeight);
        LIC<3> fixedCurve({ fixedPos, fixedPos }, { timeRange[0], timeRange[1] });
        trData->setTargetTrajectory(fixedCurve, trData->getSubTrajectoryByID(baseCurveID).m_trajectoryType, baseCurveID);
        trData->m_fixedTarget = true;
		if (config->getAnimationType()==AnimationType::WALKING) {
			if (config->m_fixedMovementFrame == -1) {
				config->m_fixedMovementFrame = config->getCurrentFrameIndex();
			}
		}
		else {
			config->m_fixedMovementFrame = config->getCurrentFrameIndex();
		}
    }

	void TrajectoryGenerator::generateEETrajectory(ChainIndex ikChain, IKRigConfig* config,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);
		FrameIndex currentFrame = config->getCurrentFrameIndex();
		FrameIndex nextFrame = config->getNextFrameIndex();
		float currentAnimTime = config->getAnimationTime(currentFrame);
		float nextAnimTime = config->getAnimationTime(nextFrame);
		float currentRepTime = config->getReproductionTime(currentFrame);
		float currSupportHeight = trData->getSupportHeight(currentFrame);
		glm::vec3 currentPos = trData->getSavedPosition(currentRepTime);
		if (config->getAnimationType() == AnimationType::IDLE) {
			int subTrID = trData->m_originalSubTrajectories[0].m_subTrajectoryID;
			generateFixedTrajectory(glm::vec2(currentPos), { currentRepTime, currentRepTime + config->getAnimationDuration() }, subTrID,
				currSupportHeight, trData, transformManager, staticMeshManager);
			return;
		}
		EETrajectory originalTrajectory = trData->getSubTrajectory(currentAnimTime);
		TrajectoryType trType;
		if (originalTrajectory.isDynamic()) {
			trType = TrajectoryType::DYNAMIC;
		}
		else {
			trType = TrajectoryType::STATIC;
		}
		LIC<3> baseCurve = originalTrajectory.getEECurve();

		FrameIndex initialFrame = config->getFrame(baseCurve.getTRange()[0]);
		FrameIndex finalFrame = config->getFrame(baseCurve.getTRange()[1]);

		HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
		LIC<3> hipPosCurve = hipTrData->sampleOriginalPositions(baseCurve.getTRange()[0], baseCurve.getTRange()[1]);

		// llevar a reproduction time
		baseCurve.offsetTValues(-currentAnimTime);
		baseCurve.offsetTValues(currentRepTime);

		if (glm::length(baseCurve.getEnd() - baseCurve.getStart()) == 0) {
			generateFixedTrajectory(glm::vec2(currentPos), { baseCurve.getTRange()[0], baseCurve.getTRange()[1] }, originalTrajectory.getSubTrajectoryID(),
				currSupportHeight, trData, transformManager, staticMeshManager);
			return;
		}

		glm::vec3 initialPos = trData->getSavedPosition(baseCurve.getTRange()[0]);
		float initialRepTime = baseCurve.getTValue(0);

        // chequear si hay info de posicion valida previa
		float strideLength = glm::distance(baseCurve.getStart(), baseCurve.getEnd());
        if (!trData->motionInitialized()) {
			glm::vec2 originalXYDirection = glm::normalize(glm::vec2(baseCurve.getEnd() - baseCurve.getStart()));
			glm::vec2 baseTargetXYDirection = glm::rotate(glm::vec2(originalXYDirection), m_ikRig->getRotationAngle());
            float referenceTime = config->getReproductionTime(currentFrame);
            glm::vec3 sampledCurveReferencePoint = baseCurve.evalCurve(referenceTime);
            float xyDistanceToStart = glm::distance(glm::vec2(baseCurve.getStart()), glm::vec2(sampledCurveReferencePoint));
            glm::vec2 currEEXYPoint = glm::vec2(currentPos);
			float supportHeightStart = trData->getSupportHeight(initialFrame);
            calcStrideStartingPoint(supportHeightStart, currEEXYPoint, xyDistanceToStart,
                baseTargetXYDirection, initialPos, transformManager, staticMeshManager);
        }

		// se calcula una direccion para el ee que logre mantener la relacion posicional con la cadera
		glm::vec2 xyHipEEOriginalDiff = hipPosCurve.getEnd() - baseCurve.getEnd();
		glm::vec2 xyHipEEUpdatedDiff = glm::rotate(xyHipEEOriginalDiff, m_ikRig->getRotationAngle());
		glm::vec2 hipCurrDir = glm::rotate(glm::vec2(m_ikRig->getFrontVector()), m_ikRig->getRotationAngle());
		glm::vec2 hipCurrXYPos = hipTrData->getSavedPosition(currentRepTime);
		if (hipTrData->getTargetPositions().inTRange(currentRepTime)) {
			hipCurrXYPos = hipTrData->getTargetPositions().evalCurve(currentRepTime);
		}
		glm::vec2 hipTrEndPredictedXYPos = hipCurrXYPos + hipCurrDir * (glm::distance(glm::vec2(hipPosCurve.evalCurve(currentAnimTime)),
			glm::vec2(hipPosCurve.getEnd())));
		glm::vec2 targetEEPos = hipTrEndPredictedXYPos - xyHipEEUpdatedDiff;
		glm::vec2 targetXYDirection = glm::normalize(targetEEPos - glm::vec2(initialPos));

		glm::vec3 finalPos;
		bool endingPosValid = calcStrideFinalPoint(trData, originalTrajectory.getSubTrajectoryID(), config,
            initialPos, strideLength, targetXYDirection, finalPos, transformManager, staticMeshManager);
        if (!endingPosValid) { // si no es posible avanzar por la elevacion del terreno
			float fixedSupportHeight = config->m_fixedMovementFrame == -1 ? currSupportHeight : trData->getSupportHeight(config->m_fixedMovementFrame);
			generateFixedTrajectory(glm::vec2(currentPos), { baseCurve.getTRange()[0], baseCurve.getTRange()[1] }, originalTrajectory.getSubTrajectoryID(),
				currSupportHeight, trData, transformManager, staticMeshManager);
            return;
        }
        baseCurve.fitEnds(initialPos, finalPos);

		if (m_strideCorrectionEnabled && originalTrajectory.isDynamic() && !trData->isTargetFixed()) {
			m_strideCorrector.correctStride(baseCurve, originalTrajectory.getEECurve(),
				m_environmentData, transformManager, staticMeshManager);
		}

        int repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
        float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
        int currSubTrID = trData->getTargetTrajectory().getSubTrajectoryID();
        int newSubTrID = originalTrajectory.getSubTrajectoryID();
        if (!trData->isTargetFixed() &&	currSubTrID == newSubTrID 
			&& trData->getTargetTrajectory().getEECurve().inTRange(transitionTime)) {
            trData->setTargetTrajectory(LIC<3>::transition(trData->getTargetTrajectory().getEECurve(),
                baseCurve, transitionTime), trType, newSubTrID);
        }
        else {
            trData->setTargetTrajectory(baseCurve, trType, newSubTrID);
        }
        trData->m_fixedTarget = false;
	}
	

    void TrajectoryGenerator::generateHipTrajectory(IKRigConfig* config,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {

        HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
        FrameIndex currentFrame = config->getCurrentFrameIndex();
		float currentRepTime = config->getReproductionTime(currentFrame);
        if (config->isMovementFixed()) {
            float initialTime_rep = config->getReproductionTime(currentFrame);
            float currFrameTime_extendedAnim = config->getAnimationTime(currentFrame);
            float nextFrameTime_extendedAnim = config->getAnimationTime(config->getNextFrameIndex());
            if (nextFrameTime_extendedAnim < currFrameTime_extendedAnim) {
                nextFrameTime_extendedAnim += config->getAnimationDuration();
            }
            glm::vec3 initialPos = hipTrData->getSavedPosition(initialTime_rep);
            // chequear si hay info de posicion valida previa
            if (!hipTrData->motionInitialized()) {
                std::pair<EEGlobalTrajectoryData*, EEGlobalTrajectoryData*> trDataPair;
                trDataPair.first = config->getEETrajectoryData(0);
                trDataPair.second = trDataPair.first->getOppositeTrajectoryData();
                glm::vec2 basePoint(initialPos);
				initialPos = glm::vec3(basePoint, calcHipAdjustedHeight(trDataPair,
                    initialTime_rep, config->getAnimationTime(currentFrame), config,
					transformManager, staticMeshManager));
            }
            LIC<3> newHipPositions({ initialPos, initialPos }, { initialTime_rep, initialTime_rep + config->getAnimationDuration() });
            hipTrData->setTargetPositions(newHipPositions);
        }
        else {
            HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
            // buscamos la curva a la que le quede mas tiempo
            float tInfLimitRep = std::numeric_limits<float>::lowest();
            float tSupLimitRep = std::numeric_limits<float>::lowest();
            EETrajectory baseEETargetTr;
            EEGlobalTrajectoryData* baseEETrData;
            for (ChainIndex i = 0; i < m_ikRig->getChainNum(); i++) {
                EEGlobalTrajectoryData* trData = config->getEETrajectoryData(i);
                EETrajectory& currTr = trData->getTargetTrajectory();
                float tCurrentSupLimit = currTr.getEECurve().getTRange()[1];
                if (tSupLimitRep < tCurrentSupLimit) {
                    tSupLimitRep = tCurrentSupLimit;
                    tInfLimitRep = currTr.getEECurve().getTRange()[0];
                    baseEETargetTr = currTr;
                    baseEETrData = trData;
                }
            }

            EETrajectory baseEEOriginalTr = baseEETrData->getSubTrajectoryByID(baseEETargetTr.getSubTrajectoryID());
            LIC<3>& baseEEOriginalCurve = baseEEOriginalTr.getEECurve();
            LIC<3>& baseEETargetCurve = baseEETrData->getTargetTrajectory().getEECurve();

			EEGlobalTrajectoryData* oppositeTrData = baseEETrData->getOppositeTrajectoryData();

            std::pair<EEGlobalTrajectoryData*, EEGlobalTrajectoryData*> trDataPair(baseEETrData, oppositeTrData);

			float tInfLimitExtendedAnim = baseEEOriginalCurve.getTRange()[0];
			float tSupLimitExtendedAnim = baseEEOriginalCurve.getTRange()[1];
            float tCurrExtendedAnim = config->getAnimationTime(currentFrame);
            while (tCurrExtendedAnim < tInfLimitExtendedAnim) { tCurrExtendedAnim += config->getAnimationDuration(); }
            while (tSupLimitExtendedAnim < tCurrExtendedAnim) { tCurrExtendedAnim -= config->getAnimationDuration(); }


            LIC<3> hipPosCurve = hipTrData->sampleOriginalPositions(tInfLimitExtendedAnim, tSupLimitExtendedAnim);
            
            glm::vec3 hipTrOriginalDirection = glm::normalize(hipPosCurve.getEnd() - hipPosCurve.getStart());

            float hipOriginalXYDistance = glm::length(glm::vec2(hipPosCurve.getEnd() - hipPosCurve.getStart()));

            FrameIndex initialFrame = config->getFrame(tInfLimitExtendedAnim);
            // calculo del punto inicial de la trayectoria
            glm::vec3 initialPos;
            // chequear si hay info de posicion valida previa
            if (hipTrData->motionInitialized()) {
				initialPos = hipTrData->getSavedPosition(tInfLimitRep);
            }
            else {
                glm::vec2 hipXYReferencePoint = hipPosCurve.evalCurve(tCurrExtendedAnim);
                float targetXYDistance = glm::distance(glm::vec2(hipPosCurve.getStart()), hipXYReferencePoint);
                glm::vec2 currXYHipPos = hipTrData->getSavedPosition(currentRepTime);
                glm::vec2 targetDirection = glm::rotate(glm::vec2(hipTrOriginalDirection), m_ikRig->getRotationAngle());
                glm::vec2 targetXYPos = currXYHipPos - targetDirection * targetXYDistance;
				initialPos = glm::vec3(glm::vec2(targetXYPos),
					calcHipAdjustedHeight(trDataPair, tInfLimitRep, tInfLimitExtendedAnim, config, transformManager, staticMeshManager));
            }        
           
			// ajuste a tiempo de reproduccion
			hipPosCurve.offsetTValues(-tInfLimitExtendedAnim + tInfLimitRep);

            // correccion de posicion y orientacion
            hipPosCurve.translate(-hipPosCurve.getStart());
            glm::fquat xyRot = glm::angleAxis(m_ikRig->getRotationAngle(), m_ikRig->getUpVector());
            hipPosCurve.rotate(xyRot);            
            hipPosCurve.translate(initialPos);            

            // ajustamos la altura de los puntos de la curva tomando en cuenta las trayectorias de los ee 
            for (int i = 1; i < hipPosCurve.getNumberOfPoints(); i++) {
                float tVal_rep = hipPosCurve.getTValue(i);
                float adjustedZ = calcHipAdjustedHeight(trDataPair, tVal_rep, tVal_rep - tInfLimitRep + tInfLimitExtendedAnim, 
                    config, transformManager, staticMeshManager);
                glm::vec3 adjustedPoint(glm::vec2(hipPosCurve.evalCurve(tVal_rep)), adjustedZ);
                hipPosCurve.setCurvePoint(i, adjustedPoint);
            }          
            

			int repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
			float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
            if (hipTrData->getTargetPositions().inTRange(transitionTime)) {
                hipTrData->setTargetPositions(LIC<3>::transition(hipTrData->getTargetPositions(), hipPosCurve, transitionTime));
            }
            else {
                hipTrData->setTargetPositions(hipPosCurve);
            }
        }
    }


	bool TrajectoryGenerator::calcStrideStartingPoint(float supportHeightStart,
		glm::vec2 xyReferencePoint, float targetDistance,
		glm::vec2 targetDirection, glm::vec3& outStrideStartPoint,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		int stepNum = 20;
		std::vector<glm::vec3> collectedPoints;
		collectedPoints.reserve(stepNum);
		for (int i = 1; i <= stepNum; i++) {
			glm::vec2 testPoint = xyReferencePoint - targetDirection * targetDistance * ((float)i / stepNum);
			float calcHeight = m_environmentData.getTerrainHeight(testPoint,
				transformManager, staticMeshManager);
			collectedPoints.push_back(glm::vec3(testPoint, calcHeight));
		}
		float minDistDiff = std::numeric_limits<float>::max();
		glm::vec3 floorReferencePoint = glm::vec3(xyReferencePoint, m_environmentData.getTerrainHeight(xyReferencePoint,
			transformManager, staticMeshManager));
		glm::vec3 selectedStartPoint(std::numeric_limits<float>::max());
		for (int i = 0; i < collectedPoints.size(); i++) {
			float distDiff = std::abs(glm::distance(floorReferencePoint, collectedPoints[i]) - targetDistance);
			if (distDiff < minDistDiff) {
				minDistDiff = distDiff;
				selectedStartPoint = collectedPoints[i];
			}
		}
		selectedStartPoint[2] += supportHeightStart;
		outStrideStartPoint = selectedStartPoint;
		return true;
	}
    
	bool TrajectoryGenerator::calcStrideFinalPoint(EEGlobalTrajectoryData* baseTrajectoryData, int baseTrajecoryID,
		IKRigConfig* config,
		glm::vec3 startingPoint, float targetDistance, 
		glm::vec2 targetDirection, glm::vec3& outStrideFinalPoint,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		int stepNum = 20;
		EETrajectory baseEETr = baseTrajectoryData->getSubTrajectoryByID(baseTrajecoryID);
		float supportHeightStart = baseEETr.getEECurve().getStart()[2];
		float supportHeightEnd = baseEETr.getEECurve().getEnd()[2];
		std::vector<glm::vec3> collectedPoints;
		collectedPoints.reserve(stepNum);
		for (int i = 1; i <= stepNum; i++) {
			glm::vec2 testPoint = glm::vec2(startingPoint) + targetDirection * targetDistance * ((float)i / stepNum);
			float supportHeight = funcUtils::lerp(supportHeightStart, supportHeightEnd, (float)i / stepNum);
			float calcHeight = supportHeight + m_environmentData.getTerrainHeight(testPoint, transformManager, staticMeshManager);
			collectedPoints.push_back(glm::vec3(testPoint, calcHeight));
		}
		float minDistDiff = std::numeric_limits<float>::max();
		glm::vec3 selectedFinalPoint(std::numeric_limits<float>::max());
		for (int i = 0; i < collectedPoints.size(); i++) {
			float distance = glm::distance(startingPoint, collectedPoints[i]);
			if (abs(targetDistance - distance) < minDistDiff) {
				minDistDiff = abs(targetDistance - distance);
				selectedFinalPoint = collectedPoints[i];
			}
		}
		outStrideFinalPoint = selectedFinalPoint;
		// validacion del punto escogido
		bool valid = true;
		if (m_strideValidationEnabled) {
			if (baseEETr.isDynamic()) {
				LIC<3> oppositeEECurve = baseTrajectoryData->getOppositeTrajectoryData()->getTargetTrajectory().getEECurve();
				FrameIndex currentFrame = config->getCurrentFrameIndex();
				float currentRepTime = config->getReproductionTime(currentFrame);
				bool oppositeDataAvailable = oppositeEECurve.inTRange(currentRepTime);
				if (oppositeDataAvailable && (baseTrajectoryData->m_fixedTarget && config->getCurrentFrameIndex() == config->getFixedMovementFrame()
					|| !baseTrajectoryData->m_fixedTarget)) {
					float currentOppositeZ = oppositeEECurve.evalCurve(currentRepTime)[2];
					float candidateEEZ = selectedFinalPoint[2];
					float glblLegLenght = m_ikRig->getRigHeight() * m_ikRig->getRigScale() / 2;
					valid = abs(currentOppositeZ - candidateEEZ) < glblLegLenght * 0.65f;
				}
				else if (baseTrajectoryData->m_fixedTarget) {
					valid = false;
				}			
			}
			else {
				// las trayectorias estaticas se calculan siempre
				valid = true;
				if (targetDistance * 0.3f < minDistDiff) {
					outStrideFinalPoint = startingPoint + glm::vec3(targetDirection, 0) * targetDistance;
				}
			}
		}	
		return valid;
		
	}


	float TrajectoryGenerator::calcHipAdjustedHeight(std::pair<EEGlobalTrajectoryData*, EEGlobalTrajectoryData*> oppositeTrajectories,
        float targetCurvesTime_rep,
		float originalCurvesTime_extendedAnim, IKRigConfig* config,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {

        
		HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
        float avgFrameDuration = config->getAnimationDuration() / config->getFrameNum();
		LIC<3> hipTrCurve = hipTrData->sampleOriginalPositions(originalCurvesTime_extendedAnim,	
            originalCurvesTime_extendedAnim + avgFrameDuration);
        float hipOriginalZ = hipTrCurve.evalCurve(originalCurvesTime_extendedAnim)[2];

        // distancias originales de ee's con cadera
        std::vector<float> origZDiffs(2);
        int baseID1 = oppositeTrajectories.first->getTargetTrajectory().getSubTrajectoryID();
        float originalZ1 = oppositeTrajectories.first->getSubTrajectoryByID(baseID1).getEECurve().evalCurve(originalCurvesTime_extendedAnim)[2];
        int baseID2 = oppositeTrajectories.second->getTargetTrajectory().getSubTrajectoryID();
        float originalZ2 = oppositeTrajectories.second->getSubTrajectoryByID(baseID2).getEECurve().evalCurve(originalCurvesTime_extendedAnim)[2];
        origZDiffs[0] = hipOriginalZ - originalZ1;
        origZDiffs[1] = hipOriginalZ - originalZ2;

		std::vector<float> targetPointsZ(2);
        targetPointsZ[0] = oppositeTrajectories.first->getTargetTrajectory().getEECurve().evalCurve(targetCurvesTime_rep)[2];
        targetPointsZ[1] = oppositeTrajectories.second->getTargetTrajectory().getEECurve().evalCurve(targetCurvesTime_rep)[2];
		float zValue = std::min(targetPointsZ[0], targetPointsZ[1]);

		// valor mas bajo de z para las tr actuales de los ee
		float zDelta = m_ikRig->getRigHeight() * m_ikRig->getRigScale() / 500;
		std::vector<bool> conditions(2, true);
		int maxSteps = 1000;
		int steps = 0;
		while (funcUtils::conditionVector_AND(conditions) && steps < maxSteps) {
			zValue += zDelta;
            std::vector<float> currZDiffs(2);
			for (int i = 0; i < 2; i++) {
				currZDiffs[i] = zValue - targetPointsZ[i];
				conditions[i] = currZDiffs[i] < origZDiffs[i];
			}

			steps += 1;
		}
		return zValue;
	}


    void TrajectoryGenerator::buildHipTrajectory(IKRigConfig* config, std::vector<glm::vec3> const& hipGlobalPositions) {
        config->m_hipTrajectoryData.init(config);
		config->m_hipTrajectoryData.m_originalPositions = LIC<3>(hipGlobalPositions, config->m_timeStamps);
    }

    void TrajectoryGenerator::buildEETrajectories(IKRigConfig* config, 
        std::vector<std::vector<bool>> supportFramesPerChain, 
        std::vector<std::vector<glm::vec3>> globalPositionsPerChain,
        std::vector<ChainIndex> oppositePerChain) {
        int chainNum = supportFramesPerChain.size();
        int frameNum = config->getFrameNum();
		std::vector<int> connectedCurveIndexes(chainNum, -1);
		std::vector<bool> continueTrajectory(chainNum, false);
        int subTrajectoryID = 0;
        for (ChainIndex i = 0; i < chainNum; i++) {
            config->m_eeTrajectoryData[i].init(config, &(config->m_eeTrajectoryData[oppositePerChain[i]]));
        }
		for (int i = 0; i < chainNum; i++) {
			std::vector<EETrajectory> subTrajectories;
			if (config->getAnimationType() == AnimationType::IDLE) {
				glm::vec3 staticPos = globalPositionsPerChain[i][0];
				LIC<3> staticTr({ staticPos, staticPos }, { config->getAnimationTime(0), config->getAnimationTime(frameNum - 1) });
				subTrajectories.push_back(EETrajectory(staticTr, TrajectoryType::STATIC, 0));
				for (int j = 0; j < frameNum; j++) { config->m_eeTrajectoryData[i].m_supportHeights[j] = globalPositionsPerChain[i][0][2]; }
			}
			else {
                bool allStatic = funcUtils::conditionVector_AND(supportFramesPerChain[i]);
                MONA_ASSERT(!allStatic, "TrajectoryGenerator: A moving animation must have at least one dynamic trajectory per chain.");
				// encontrar primer punto de interes (dinamica luego de uno estatico)
				FrameIndex curveStartFrame = -1;
				for (int j = 1; j < frameNum; j++) {
					if (supportFramesPerChain[i][j - 1] && !supportFramesPerChain[i][j]) {
						curveStartFrame = j;
						break;
					}
				}
				MONA_ASSERT(curveStartFrame != -1, "TrajectoryGenerator: There must be at least one support frame per ee trajectory.");
				std::vector<std::pair<int, int>> shInterpolationLimits;
				float supportHeight;
				int j = curveStartFrame;
				FrameIndex currFrame;
				while (j < frameNum + curveStartFrame) {
					currFrame = j % frameNum;
					FrameIndex initialFrame = 0 < currFrame ? currFrame - 1 : frameNum - 1;
					bool baseFrameType = supportFramesPerChain[i][currFrame];
					TrajectoryType trType = baseFrameType ? TrajectoryType::STATIC : TrajectoryType::DYNAMIC;
					supportHeight = globalPositionsPerChain[i][initialFrame][2];
					config->m_eeTrajectoryData[i].m_supportHeights[initialFrame] = supportHeight;
					if (trType == TrajectoryType::DYNAMIC) {
						shInterpolationLimits.push_back({ j - 1, j });
					}
					std::vector<glm::vec3> curvePoints_1 = { globalPositionsPerChain[i][initialFrame] };
					std::vector<float> tValues_1 = { config->getAnimationTime(initialFrame) };
					std::vector<glm::vec3> curvePoints_2;
					std::vector<float> tValues_2;
					std::vector<glm::vec3>* selectedCPArr = &curvePoints_1;
					std::vector<float>* selectedTVArr = &tValues_1;
				GATHER_POINTS:
					while (baseFrameType == supportFramesPerChain[i][currFrame]) {
						config->m_eeTrajectoryData[i].m_supportHeights[currFrame] = baseFrameType ?
							globalPositionsPerChain[i][currFrame][2] : supportHeight;
						(*selectedCPArr).push_back(globalPositionsPerChain[i][currFrame]);
						(*selectedTVArr).push_back(config->getAnimationTime(currFrame));
						j++;
						currFrame = j % frameNum;
						if (j == frameNum) {
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
					LIC<3> fullCurve;
					if (curvePoints_2.size() == 0) {
						fullCurve = LIC<3>(curvePoints_1, tValues_1);
					}
					else if (curvePoints_2.size() == 1) {
						connectedCurveIndexes[i] = subTrajectories.size();
						LIC<3> curve(curvePoints_1, tValues_1);
						float tDiff = tValues_2[0] + config->getAnimationDuration() - curve.getTRange()[1];
						if (0 < tDiff) {
							fullCurve = LIC<3>::connectPoint(curve, curvePoints_2[0], tDiff);
						}
						else {
							fullCurve = curve;
						}
					}
					else if (1 < curvePoints_2.size()) {
						connectedCurveIndexes[i] = subTrajectories.size();
						LIC<3> part1(curvePoints_1, tValues_1);
						LIC<3> part2(curvePoints_2, tValues_2);
						fullCurve = LIC<3>::connect(part1, part2);
					}
					subTrajectories.push_back(EETrajectory(fullCurve, trType));
					if (trType == TrajectoryType::DYNAMIC) {
						shInterpolationLimits.back().second = j;
					}
				}
				if (connectedCurveIndexes[i] != -1) {
					// falta agregar la misma curva pero al comienzo del arreglo (con otro desplazamiento temporal)
					LIC<3> connectedCurve = subTrajectories[connectedCurveIndexes[i]].getEECurve();
					TrajectoryType connectedTrType = subTrajectories[connectedCurveIndexes[i]].isDynamic() ? TrajectoryType::DYNAMIC : TrajectoryType::STATIC;
					connectedCurve.offsetTValues(-config->getAnimationDuration());
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

				// corregimos las posiciones de la trayectoria insertada al principio (si es que la hubo)
				if (connectedCurveIndexes[i] != -1) {
					glm::vec3 targetEnd = subTrajectories[1].getEECurve().getStart();
					LIC<3>& curveToCorrect = subTrajectories[0].getEECurve();
					curveToCorrect.translate(-curveToCorrect.getEnd());
					curveToCorrect.translate(targetEnd);
				}

				// interpolamos los valores de support height para las curvas dinamicas
				for (int k = 0; k < shInterpolationLimits.size(); k++) {
					float sh1 = config->m_eeTrajectoryData[i].m_supportHeights[shInterpolationLimits[k].first % frameNum];
					float sh2 = config->m_eeTrajectoryData[i].m_supportHeights[shInterpolationLimits[k].second % frameNum];
					int minIndex = shInterpolationLimits[k].first;
					int maxIndex = shInterpolationLimits[k].second;
					if (sh1 != sh2) {
						for (int l = minIndex; l <= maxIndex; l++) {
							FrameIndex shFrame = l % frameNum;
							float shFraction = funcUtils::getFraction(minIndex, maxIndex, l);
							config->m_eeTrajectoryData[i].m_supportHeights[shFrame] = funcUtils::lerp(sh1, sh2, shFraction);
						}
					}
				}
			}

			// asignamos las sub trayectorias a la cadena correspondiente
			config->m_eeTrajectoryData[i].m_originalSubTrajectories = subTrajectories;
			for (int j = 0; j < subTrajectories.size(); j++) {
				config->m_eeTrajectoryData[i].m_originalSubTrajectories[j].m_subTrajectoryID = subTrajectoryID;
                subTrajectoryID += 1;
			}
		}
    }

    
}