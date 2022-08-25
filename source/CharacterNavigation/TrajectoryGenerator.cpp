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

    // funciones para descenso de gradiente

    // primer termino: acercar los modulos de las velocidades
    std::function<float(const std::vector<float>&, TGData*)> term1Function =
        [](const std::vector<float>& varPCoord, TGData* dataPtr)->float {
        float result = 0;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            int pIndex = dataPtr->pointIndexes[i];
            result += glm::distance2(dataPtr->varCurve->getPointVelocity(pIndex),  dataPtr->baseCurve.getPointVelocity(pIndex));
            result += glm::distance2(dataPtr->varCurve->getPointVelocity(pIndex, true), dataPtr->baseCurve.getPointVelocity(pIndex, true));
        }
        return result;
    };

    std::function<float(const std::vector<float>&, int, TGData*)> term1PartialDerivativeFunction =
        [](const std::vector<float>& varPCoord, int varIndex, TGData* dataPtr)->float {
        int D = 3;
        int pIndex = dataPtr->pointIndexes[varIndex / D];
        int coordIndex = varIndex % D;
		float t_kPrev = dataPtr->varCurve->getTValue(pIndex - 1);
		float t_kCurr = dataPtr->varCurve->getTValue(pIndex);
		float t_kNext = dataPtr->varCurve->getTValue(pIndex + 1);
		glm::vec3 lVel = dataPtr->varCurve->getPointVelocity(pIndex);
		glm::vec3 rVel = dataPtr->varCurve->getPointVelocity(pIndex, true);
        glm::vec3 baseLVel = dataPtr->baseCurve.getPointVelocity(pIndex);
		glm::vec3 baseRVel = dataPtr->baseCurve.getPointVelocity(pIndex, true);
        float result = 0;
        result += 2 * (lVel[coordIndex] - baseLVel[coordIndex]) * (1 / (t_kCurr - t_kPrev));
        result += 2 * (rVel[coordIndex] - baseRVel[coordIndex]) * (-1 / (t_kNext - t_kCurr));
        return result;
    };

    std::function<void(std::vector<float>&, TGData*, std::vector<float>&, int)>  postDescentStepCustomBehaviour =
        [](std::vector<float>& varPCoord, TGData* dataPtr, std::vector<float>& argsRawDelta, int varIndex_progressive)->void {
        glm::vec3 newPos;
        int D = 3;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            for (int j = 0; j < D; j++) {
                if (varPCoord[i * D + j] <= dataPtr->minValues[i * D + j]) {
                    varPCoord[i * D + j] = dataPtr->minValues[i * D + j];
                    argsRawDelta[i * D + j] *= 0.1;
                }
                newPos[j] = varPCoord[i * D + j];
            }
            int pIndex = dataPtr->pointIndexes[i];
            dataPtr->varCurve->setCurvePoint(pIndex, newPos);
        }
    };


    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig) {
        m_ikRig = ikRig;
    }

    void TrajectoryGenerator::init() {
        FunctionTerm<TGData> term1(term1Function, term1PartialDerivativeFunction);
        m_gradientDescent = GradientDescent<TGData>({ term1 }, 0, &m_tgData, postDescentStepCustomBehaviour);
        m_tgData.descentRate = 1 / pow(10, 3);
        m_tgData.maxIterations = 600;
        m_tgData.targetPosDelta = 1 /pow(10, 6);
        m_gradientDescent.setTermWeight(0, 1.0f);
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
        // si una trayectoria es fija, las demas tambien deberan serlo
        if (config->getAnimationType() == AnimationType::WALKING) {
            std::vector<bool> fixedTrajectories;
            for (ChainIndex i = 0; i < m_ikRig->getChainNum(); i++) {
                fixedTrajectories.push_back(config->getEETrajectoryData(i)->isTargetFixed());
            }
            if (funcUtils::conditionVector_OR(fixedTrajectories)) {
                FrameIndex currentFrame = config->getCurrentFrameIndex();
                float currentRepTime = config->getReproductionTime(currentFrame);
                for (ChainIndex i = 0; i < m_ikRig->getChainNum(); i++) {
                    EEGlobalTrajectoryData* trData = config->getEETrajectoryData(i);
                    float currSupportHeight = trData->getSupportHeight(currentFrame);
                    glm::vec3 currentPos = trData->getSavedPosition(currentFrame);
                    int trID = trData->getTargetTrajectory().getSubTrajectoryID();
                    generateFixedTrajectory(glm::vec2(currentPos), { currentRepTime, currentRepTime + config->getAnimationDuration() }, trID,
                        currSupportHeight, i, config, transformManager, staticMeshManager);
                }
            }
        
        }

		// queda setear la trayectoria de la cadera
		generateHipTrajectory(config, transformManager, staticMeshManager);
	}

    void TrajectoryGenerator::generateFixedTrajectory(glm::vec2 basePos,
        glm::vec2 timeRange, int baseCurveID, float supportHeight,
        ChainIndex ikChain, IKRigConfig* config,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);
        float calcHeight = m_environmentData.getTerrainHeight(glm::vec2(basePos), transformManager, staticMeshManager);
        glm::vec3 fixedPos(basePos, calcHeight + supportHeight);
        LIC<3> fixedCurve({ fixedPos, fixedPos }, { timeRange[0], timeRange[1] });
        trData->setTargetTrajectory(fixedCurve, TrajectoryType::STATIC, baseCurveID);
        trData->m_fixedTarget = true;
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
        glm::vec3 currentPos = trData->getSavedPosition(currentFrame);
		if (config->getAnimationType() == AnimationType::IDLE) {
            int subTrID = trData->m_originalSubTrajectories[0].m_subTrajectoryID;
            generateFixedTrajectory(glm::vec2(currentPos), { currentRepTime, currentRepTime + config->getAnimationDuration() }, subTrID,
                currSupportHeight ,ikChain, config, transformManager, staticMeshManager);
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
        LIC<3>& baseCurve = originalTrajectory.getEECurve();

        FrameIndex initialFrame = config->getFrame(baseCurve.getTRange()[0]);
        FrameIndex finalFrame = config->getFrame(baseCurve.getTRange()[1]);

        // llevar a reproduction time
        baseCurve.offsetTValues(-currentAnimTime);
        baseCurve.offsetTValues(currentRepTime);

		if (glm::length(baseCurve.getEnd() - baseCurve.getStart()) == 0) {
			generateFixedTrajectory(glm::vec2(currentPos), { baseCurve.getTRange()[0], baseCurve.getTRange()[1] }, originalTrajectory.getSubTrajectoryID(),
				currSupportHeight, ikChain, config, transformManager, staticMeshManager);
			return;
		}

        glm::vec3 initialPos = trData->getSavedPosition(initialFrame);
        float initialRepTime = baseCurve.getTValue(0);
		
		glm::vec2 originalXYDirection = glm::normalize(glm::vec2(baseCurve.getEnd() - baseCurve.getStart()));
		glm::vec2 targetXYDirection = glm::rotate(glm::vec2(originalXYDirection), m_ikRig->getRotationAngle());
		
        // chequear si hay info de posicion valida previa
        if (!(trData->isSavedDataValid(initialFrame) && trData->getTargetTrajectory().getEECurve().inTRange(initialRepTime))) {
            float referenceTime = config->getReproductionTime(currentFrame);
            glm::vec3 sampledCurveReferencePoint = baseCurve.evalCurve(referenceTime);
            float xyDistanceToStart = glm::distance(glm::vec2(baseCurve.getStart()), glm::vec2(sampledCurveReferencePoint));
            glm::vec2 currEEXYPoint = glm::vec2(currentPos);
            float supportHeight = trData->getSupportHeight(initialFrame);
            initialPos = calcStrideStartingPoint(supportHeight, currEEXYPoint, xyDistanceToStart,
                targetXYDirection, 4, transformManager, staticMeshManager);
        }
        
        float targetDistance = glm::distance(baseCurve.getStart(), baseCurve.getEnd());
        float supportHeightStart = trData->getSupportHeight(initialFrame);
		float supportHeightEnd = trData->getSupportHeight(finalFrame);
        std::vector<glm::vec3> strideData = calcStrideData(supportHeightStart, supportHeightEnd,
            initialPos, targetDistance, targetXYDirection, 8, transformManager, staticMeshManager);
        if (strideData.size() == 0) { // si no es posible avanzar por la elevacion del terreno
			generateFixedTrajectory(glm::vec2(currentPos), { baseCurve.getTRange()[0], baseCurve.getTRange()[1] }, originalTrajectory.getSubTrajectoryID(),
				currSupportHeight, ikChain, config, transformManager, staticMeshManager);
            return;
        }
        glm::vec3 finalPos = strideData.back();
        baseCurve.fitEnds(initialPos, finalPos);

        // setear los minimos de altura y aplicar el descenso de gradiente
        m_tgData.pointIndexes.clear();
        m_tgData.pointIndexes.reserve(baseCurve.getNumberOfPoints());
        m_tgData.minValues.clear();
        m_tgData.minValues.reserve(baseCurve.getNumberOfPoints() * 3);
        for (int i = 1; i < baseCurve.getNumberOfPoints() - 1; i++) {
            m_tgData.pointIndexes.push_back(i);
        }
        std::vector<std::pair<int, int>> curvePointIndex_stridePointIndex;
        std::vector<int> baseCurveIndices;
        baseCurveIndices.reserve(baseCurve.getNumberOfPoints());
        std::vector<int> strideDataIndices;
        strideDataIndices.reserve(strideData.size());
        for (int i = 0; i < m_tgData.pointIndexes.size(); i++) { baseCurveIndices.push_back(m_tgData.pointIndexes[i]); }
        for (int i = 0; i < strideData.size(); i++) { strideDataIndices.push_back(i); }
        while (0 < strideDataIndices.size() && 0 < baseCurveIndices.size()) {
            float minDistance = std::numeric_limits<float>::max();
            int minBaseCurveIndex = -1;
            int savedI = -1;
            int strideDataIndex = strideDataIndices[0];
            for (int i = 0; i < baseCurveIndices.size(); i++) {
                float baseCurveIndex = baseCurveIndices[i];
                float currDistance = glm::distance2(glm::vec2(strideData[strideDataIndex]), glm::vec2(baseCurve.getCurvePoint(baseCurveIndex)));
                if (currDistance < minDistance) {
                    minDistance = currDistance;
                    minBaseCurveIndex = baseCurveIndex;
                    savedI = i;
                }
            }
            curvePointIndex_stridePointIndex.push_back({ minBaseCurveIndex, strideDataIndex });
            baseCurveIndices.erase(baseCurveIndices.begin() + savedI);
            strideDataIndices.erase(strideDataIndices.begin());
        }

        m_tgData.minValues = std::vector<float>(m_tgData.pointIndexes.size() * 3, std::numeric_limits<float>::lowest());
        std::vector<int> assignedTGDataPointIndexes;
        for (int i = 0; i < curvePointIndex_stridePointIndex.size(); i++) {
            int curvePointIndex = curvePointIndex_stridePointIndex[i].first;
            int tgDataIndex = -1;
            for (int j = 0; j < m_tgData.pointIndexes.size(); j++) {
                if (curvePointIndex == m_tgData.pointIndexes[j]) {
                    tgDataIndex = j;
                    break;
                }
            }
            if (tgDataIndex != -1) {
                int stridePointIndex = curvePointIndex_stridePointIndex[i].second;
                m_tgData.minValues[tgDataIndex * 3 + 2] = strideData[stridePointIndex][2];
                assignedTGDataPointIndexes.push_back(tgDataIndex);
            }
        }
        if (assignedTGDataPointIndexes[0] != 0) {
            m_tgData.minValues[2] = strideData[0][2];
            assignedTGDataPointIndexes.insert(assignedTGDataPointIndexes.begin(), 0);
        }
        if (assignedTGDataPointIndexes.back() != m_tgData.pointIndexes.size() - 1) {
            m_tgData.minValues.back() = strideData.back()[2];
            assignedTGDataPointIndexes.push_back(m_tgData.pointIndexes.size() - 1);
        }


        for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
            int minLerpIndex = -1;
            int maxLerpIndex = -1;
            float minVal;
            float maxVal;
            int currIndex = i;
            for (int j = 0; j < assignedTGDataPointIndexes.size() - 1; j++) {
                if (assignedTGDataPointIndexes[j] < currIndex && currIndex < assignedTGDataPointIndexes[j + 1]) {
                    minLerpIndex = assignedTGDataPointIndexes[j];
                    maxLerpIndex = assignedTGDataPointIndexes[j + 1];
                    minVal = m_tgData.minValues[minLerpIndex * 3 + 2];
                    maxVal = m_tgData.minValues[maxLerpIndex * 3 + 2];
                    break;
                }
            }
            if (minLerpIndex != -1) {
                m_tgData.minValues[currIndex * 3 + 2] = funcUtils::lerp(minVal, maxVal, funcUtils::getFraction(minLerpIndex, maxLerpIndex, currIndex));
            }

        }

        // valores iniciales y curva base
        std::vector<float> initialArgs(m_tgData.pointIndexes.size() * 3);
        for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
            int pIndex = m_tgData.pointIndexes[i];
            for (int j = 0; j < 3; j++) {
                initialArgs[i * 3 + j] = baseCurve.getCurvePoint(pIndex)[j];
            }
        }
        m_tgData.baseCurve = baseCurve;
        m_tgData.varCurve = &baseCurve;

        m_gradientDescent.setArgNum(initialArgs.size());
        m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, m_tgData.targetPosDelta, initialArgs);

        float repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
        float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
        int currSubTrID = trData->getTargetTrajectory().getSubTrajectoryID();
        int newSubTrID = originalTrajectory.getSubTrajectoryID();
        if (currSubTrID == newSubTrID && trData->getTargetTrajectory().getEECurve().inTRange(transitionTime)) {
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

        std::vector<bool> fixedTrajectories;
        for (ChainIndex i = 0; i < m_ikRig->getChainNum(); i++) {
            fixedTrajectories.push_back(config->getEETrajectoryData(i)->isTargetFixed());
        }

        HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        if (funcUtils::conditionVector_OR(fixedTrajectories)) {
            float initialTime_rep = config->getReproductionTime(currentFrame);
            float currFrameTime_extendedAnim = config->getAnimationTime(currentFrame);
            float nextFrameTime_extendedAnim = config->getAnimationTime(config->getNextFrameIndex());
            if (nextFrameTime_extendedAnim < currFrameTime_extendedAnim) {
                nextFrameTime_extendedAnim += config->getAnimationDuration();
            }
            glm::vec3 initialPos = hipTrData->getSavedPosition(currentFrame);
            // chequear si hay info de posicion valida previa
            if (!(hipTrData->isSavedDataValid(currentFrame) && hipTrData->getTargetPositions().inTRange(initialTime_rep))) {
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
            if (hipTrData->isSavedDataValid(initialFrame) && hipTrData->getTargetPositions().inTRange(tInfLimitRep)) {
				initialPos = hipTrData->getSavedPosition(initialFrame);
            }
            else {
                glm::vec2 hipXYReferencePoint = hipPosCurve.evalCurve(tCurrExtendedAnim);
                float targetXYDistance = glm::distance(glm::vec2(hipPosCurve.getStart()), hipXYReferencePoint);
                glm::vec2 currXYHipPos = hipTrData->getSavedPosition(currentFrame);
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
            

			float repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
			float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
            if (hipTrData->getTargetPositions().inTRange(transitionTime)) {
                hipTrData->setTargetPositions(LIC<3>::transition(hipTrData->getTargetPositions(), hipPosCurve, transitionTime));
            }
            else {
                hipTrData->setTargetPositions(hipPosCurve);
            }
        }
    }


	glm::vec3 TrajectoryGenerator::calcStrideStartingPoint(float supportHeight,
		glm::vec2 xyReferencePoint, float targetDistance,
		glm::vec2 targetDirection, int stepNum,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		std::vector<glm::vec3> collectedPoints;
		collectedPoints.reserve(stepNum);
		for (int i = 1; i <= stepNum; i++) {
			glm::vec2 testPoint = xyReferencePoint - targetDirection * targetDistance * ((float)i / stepNum);
			float calcHeight = m_environmentData.getTerrainHeight(testPoint,
				transformManager, staticMeshManager);
			collectedPoints.push_back(glm::vec3(testPoint, calcHeight));
		}
		float minDistanceDiff = std::numeric_limits<float>::max();
		glm::vec3 floorReferencePoint = glm::vec3(xyReferencePoint, m_environmentData.getTerrainHeight(xyReferencePoint,
			transformManager, staticMeshManager));
		glm::vec3 closest = collectedPoints[0];
		for (int i = 0; i < collectedPoints.size(); i++) {
			float distDiff = std::abs(glm::distance(floorReferencePoint, collectedPoints[i]) - targetDistance);
			if (distDiff < minDistanceDiff) {
				minDistanceDiff = distDiff;
				closest = collectedPoints[i];
			}
		}
        closest[2] += supportHeight;
		return closest;
	}
    

	std::vector<glm::vec3> TrajectoryGenerator::calcStrideData(float supportHeightStart, float supportHeightEnd,
        glm::vec3 startingPoint, float targetDistance,
		glm::vec2 targetDirection, int stepNum,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		std::vector<glm::vec3> collectedPoints;
		collectedPoints.reserve(stepNum);
		for (int i = 1; i <= stepNum; i++) {
			glm::vec2 testPoint = glm::vec2(startingPoint) + targetDirection * targetDistance * ((float)i / stepNum);
            float supportHeight = funcUtils::lerp(supportHeightStart, supportHeightEnd, (float) i / stepNum);
			float calcHeight = supportHeight + m_environmentData.getTerrainHeight(testPoint, transformManager, staticMeshManager);
			collectedPoints.push_back(glm::vec3(testPoint, calcHeight));
		}
        // validacion de los puntos
		std::vector<glm::vec3> strideDataPoints;
		strideDataPoints.reserve(stepNum);
		float previousDistance = 0;
        float epsilon = targetDistance * 0.005;
        float minDistDiff = std::numeric_limits<float>::max();
        int finalPointIndex = -1;
		for (int i = 0; i < collectedPoints.size(); i++) {
			float distance = glm::distance(startingPoint, collectedPoints[i]);
			if (abs(targetDistance - distance) < minDistDiff) {
                minDistDiff = abs(targetDistance - distance);
                finalPointIndex = i;
			}
		}
        for (int i = 0; i <= finalPointIndex; i++) {
            strideDataPoints.push_back(collectedPoints[i]);
        }

		return strideDataPoints;
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