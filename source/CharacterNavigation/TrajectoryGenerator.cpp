#include "TrajectoryGenerator.hpp"
#include "../Core/GlmUtils.hpp"
#include "../Core/FuncUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "IKRig.hpp"

namespace Mona{

    // funciones para descenso de gradiente

    // primer termino: acercar las direcciones de las velocidades
	std::function<float(const std::vector<float>&, TGData*)> term1Function =
		[](const std::vector<float>& varPCoords, TGData* dataPtr)->float {
		float result = 0;
		for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
			int pIndex = dataPtr->pointIndexes[i];
			float tVal = dataPtr->varCurve->getTValue(pIndex);
            glm::vec3 lVel = dataPtr->varCurve->getVelocity(tVal);
            glm::vec3 rVel = dataPtr->varCurve->getVelocity(tVal, true);
            glm::vec3 baseLVel = dataPtr->baseCurve.getVelocity(tVal);
            glm::vec3 baseRVel = dataPtr->baseCurve.getVelocity(tVal, true);
            result += glm::distance2(lVel / glm::length(lVel), baseLVel / glm::length(baseLVel));
            result += glm::distance2(rVel / glm::length(rVel), baseRVel / glm::length(baseRVel));
		}
		return dataPtr->alphaValue * result;
	};

	std::function<float(const std::vector<float>&, int, TGData*)> term1PartialDerivativeFunction =
		[](const std::vector<float>& varPCoords, int varIndex, TGData* dataPtr)->float {
		int D = 3;
		int pIndex = dataPtr->pointIndexes[varIndex / D];
		int coordIndex = varIndex % D;
        float t_kPrev = dataPtr->varCurve->getTValue(pIndex - 1);
        float t_kCurr = dataPtr->varCurve->getTValue(pIndex);
        float t_kNext = dataPtr->varCurve->getTValue(pIndex + 1);
		glm::vec3 lVel = dataPtr->varCurve->getVelocity(t_kCurr);
        float safeLVelLength = (glm::length(lVel) > 0 ? glm::length(lVel) : pow(10, -4));
		glm::vec3 rVel = dataPtr->varCurve->getVelocity(t_kCurr, true);
        float safeRVelLength = (glm::length(rVel) > 0 ? glm::length(rVel) : pow(10, -4));
		glm::vec3 baseLVel = dataPtr->baseCurve.getVelocity(t_kCurr);
        float safeBaseLVelLength = (glm::length(baseLVel) > 0 ? glm::length(baseLVel) : pow(10, -4));
		glm::vec3 baseRVel = dataPtr->baseCurve.getVelocity(t_kCurr, true);
        float safeBaseRVelLength = (glm::length(baseRVel) > 0 ? glm::length(baseRVel) : pow(10, -4));
        float result = 0;
        result += 2 * (lVel[coordIndex] / safeLVelLength - baseLVel[coordIndex] / safeBaseLVelLength)
            * ((1 / (t_kCurr - t_kPrev)) * safeLVelLength - lVel[coordIndex] * (lVel[coordIndex]/safeLVelLength) * (1 / (t_kCurr - t_kPrev)))/
            pow(safeLVelLength, 2);
        result += 2 * (rVel[coordIndex] / safeRVelLength - baseRVel[coordIndex] / safeBaseRVelLength)
            * ((-1 / (t_kNext - t_kCurr)) * safeRVelLength - rVel[coordIndex] * (rVel[coordIndex] / safeRVelLength) * (-1 / (t_kNext - t_kCurr))) /
            pow(safeRVelLength, 2);
		return dataPtr->alphaValue * result;
	};

    // segundo termino: acercar los modulos de las velocidades
    std::function<float(const std::vector<float>&, TGData*)> term2Function =
        [](const std::vector<float>& varPCoord, TGData* dataPtr)->float {
        float result = 0;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            int pIndex = dataPtr->pointIndexes[i];
            float tVal = dataPtr->varCurve->getTValue(pIndex);
            result += glm::distance2(dataPtr->varCurve->getVelocity(tVal),  dataPtr->baseCurve.getVelocity(tVal));
            result += glm::distance2(dataPtr->varCurve->getVelocity(tVal, true), dataPtr->baseCurve.getVelocity(tVal, true));
        }
        return dataPtr->betaValue*result;
    };

    std::function<float(const std::vector<float>&, int, TGData*)> term2PartialDerivativeFunction =
        [](const std::vector<float>& varPCoord, int varIndex, TGData* dataPtr)->float {
        int D = 3;
        int pIndex = dataPtr->pointIndexes[varIndex / D];
        int coordIndex = varIndex % D;
		float t_kPrev = dataPtr->varCurve->getTValue(pIndex - 1);
		float t_kCurr = dataPtr->varCurve->getTValue(pIndex);
		float t_kNext = dataPtr->varCurve->getTValue(pIndex + 1);
		glm::vec3 lVel = dataPtr->varCurve->getVelocity(t_kCurr);
		glm::vec3 rVel = dataPtr->varCurve->getVelocity(t_kCurr, true);
        glm::vec3 baseLVel = dataPtr->baseCurve.getVelocity(t_kCurr);
		glm::vec3 baseRVel = dataPtr->baseCurve.getVelocity(t_kCurr, true);
        float result = 0;
        result += 2 * (lVel[coordIndex] - baseLVel[coordIndex]) * (1 / (t_kCurr - t_kPrev));
        result += 2 * (rVel[coordIndex] - baseRVel[coordIndex]) * (-1 / (t_kNext - t_kCurr));
        return dataPtr->betaValue*result;
    };

    std::function<void(std::vector<float>&, TGData*, std::vector<float>&)>  postDescentStepCustomBehaviour =
        [](std::vector<float>& varPCoord, TGData* dataPtr, std::vector<float>& argsRawDelta)->void {
        glm::vec3 newPos;
        int D = 3;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            for (int j = 0; j < D; j++) {
                if (varPCoord[i * D + j] <= dataPtr->minValues[i * D + j]) {
                    varPCoord[i * D + j] = dataPtr->minValues[i * D + j];
                    argsRawDelta[i * D + j] *= 0.5;
                }
                newPos[j] = varPCoord[i * D + j];
            }
            int pIndex = dataPtr->pointIndexes[i];
            dataPtr->varCurve->setCurvePoint(pIndex, newPos);
        }
    };


    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains) {
        m_ikRig = ikRig;
        m_ikChains = ikChains;
    }

    void TrajectoryGenerator::init() {
        FunctionTerm<TGData> term1(term1Function, term1PartialDerivativeFunction);
        FunctionTerm<TGData> term2(term2Function, term2PartialDerivativeFunction);
        m_gradientDescent = GradientDescent<TGData>({ term2 }, 0, &m_tgData, postDescentStepCustomBehaviour);
        m_tgData.descentRate = m_ikRig->getRigHeight()/pow(10,5);
        m_tgData.maxIterations = 600;
        m_tgData.alphaValue = 0.8;
        m_tgData.betaValue = 1.0;
        m_tgData.targetPosDelta = m_ikRig->getRigHeight()/pow(10, 6);
    }

	void TrajectoryGenerator::generateNewTrajectories(AnimationIndex animIndex,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		// las trayectorias anteriores siempre deben llegar hasta el currentFrame

		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
        float xyRotationAngle = 0;
        glm::vec3 originalDir = glm::vec3(config->getHipTrajectoryData()->getOriginalFrontVector(), 0);
        glm::vec3 newDir = glm::vec3(m_ikRig->getFrontVector(), 0);
        if (originalDir != newDir) {
            float crossVectorL = glm::length(glm::cross(originalDir, newDir));
			xyRotationAngle = glm::asin(crossVectorL);
        }        

		for (int i = 0; i < m_ikChains.size(); i++) {
			ChainIndex ikChain = m_ikChains[i];
			JointIndex eeIndex = m_ikRig->getIKChain(ikChain)->getJoints().back();
			generateEETrajectory(ikChain, config, xyRotationAngle, transformManager, staticMeshManager);
		}

		// queda setear la trayectoria de la cadera
		generateHipTrajectory(config, xyRotationAngle, transformManager, staticMeshManager);
	}

	void TrajectoryGenerator::generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);
		FrameIndex currentFrame = config->getCurrentFrameIndex();
		float currentAnimTime = config->getAnimationTime(currentFrame);
		float currentRepTime = config->getReproductionTime(currentFrame);
		if (config->getAnimationType() == AnimationType::IDLE) {
			glm::vec3 globalEEPos = trData->getSavedPosition(currentFrame);
			LIC<3> baseCurve({ globalEEPos, globalEEPos }, { currentRepTime, currentRepTime + config->getAnimationDuration() });
			generateStaticTrajectory(EETrajectory(baseCurve, TrajectoryType::STATIC), ikChain, config, xyMovementRotAngle,
				transformManager, staticMeshManager);
			return;
		}
		EETrajectory originalTrajectory = trData->getSubTrajectory(currentAnimTime);
		if (originalTrajectory.isDynamic()) {
			generateDynamicTrajectory(originalTrajectory, ikChain, config, xyMovementRotAngle, transformManager, staticMeshManager);
		}
		else {
			generateStaticTrajectory(originalTrajectory, ikChain, config, xyMovementRotAngle, transformManager, staticMeshManager);
		}
	}

    void TrajectoryGenerator::generateStaticTrajectory(EETrajectory baseTrajectory,
        ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {

        EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);
        LIC<3>& baseCurve = baseTrajectory.getEECurve();

		FrameIndex currentFrame = config->getCurrentFrameIndex();
		FrameIndex initialFrame = config->getFrame(baseCurve.getTValue(0));
		// llevar a reproduction time
		float currentAnimTime = config->getAnimationTime(currentFrame);
		float currentRepTime = config->getReproductionTime(currentFrame);
		baseCurve.offsetTValues(-currentAnimTime);
		baseCurve.offsetTValues(currentRepTime);
        
        int repCountOffset = initialFrame <= currentFrame ? 0 : -1;
        float initialTime = config->getReproductionTime(initialFrame, repCountOffset);
        glm::vec3 initialPos = trData->getSavedPosition(initialFrame);
        // chequear si hay una curva previa generada
        if (!trData->getTargetTrajectory().getEECurve().inTRange(initialTime)) {
            float calcHeight = m_environmentData.getTerrainHeight(glm::vec2(initialPos), transformManager, staticMeshManager);
            initialPos = glm::vec3(glm::vec2(initialPos), trData->getSupportHeight(initialFrame) + calcHeight);
        }
		glm::vec3 originalDirection = glm::normalize(baseCurve.getEnd() - baseCurve.getStart());
		glm::vec2 targetXYDirection = glm::rotate(glm::vec2(originalDirection), xyMovementRotAngle);
        glm::vec3 targetDirection = glm::normalize(glm::vec3(targetXYDirection, originalDirection[2]));
        // correccion de posicion
        baseCurve.fitStartAndDir(initialPos, targetDirection);

		repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
		float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
		int currSubTrID = trData->getTargetTrajectory().getSubTrajectoryID();
		int newSubTrID = baseTrajectory.getSubTrajectoryID();
		if (currSubTrID == newSubTrID) {
			trData->setTargetTrajectory(LIC<3>::transition(trData->getTargetTrajectory().getEECurve(),
				baseCurve, transitionTime), TrajectoryType::STATIC, baseTrajectory.getSubTrajectoryID());
        }
        else {
            trData->setTargetTrajectory(baseCurve, TrajectoryType::STATIC, baseTrajectory.getSubTrajectoryID());
        }
    }

    void TrajectoryGenerator::generateDynamicTrajectory(EETrajectory baseTrajectory, 
        ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {

        LIC<3>& baseCurve = baseTrajectory.getEECurve();
        EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        FrameIndex initialFrame = config->getFrame(baseCurve.getTRange()[0]);
        FrameIndex finalFrame = config->getFrame(baseCurve.getTRange()[1]);

        // llevar a reproduction time
        float currentAnimTime = config->getAnimationTime(currentFrame);
        float currentRepTime = config->getReproductionTime(currentFrame);
        baseCurve.offsetTValues(-currentAnimTime);
        baseCurve.offsetTValues(currentRepTime);

        float initialRepTime = baseCurve.getTValue(0);
        glm::vec3 initialPos = trData->getSavedPosition(initialFrame);
        glm::vec3 currentPos = trData->getSavedPosition(currentFrame);
        // chequear si hay una curva previa generada
        if (!trData->getTargetTrajectory().getEECurve().inTRange(initialRepTime)) {
            float referenceTime = config->getReproductionTime(currentFrame);
            glm::vec3 sampledCurveReferencePoint = baseCurve.evalCurve(referenceTime);
            float xyDistanceToStart = glm::distance(glm::vec2(baseCurve.getStart()), glm::vec2(sampledCurveReferencePoint));
            glm::vec2 xyReferencePoint = glm::vec2(currentPos);
            glm::vec2 sampledCurveReferenceDirection = glm::vec2(sampledCurveReferencePoint) - glm::vec2(baseCurve.getStart());
            glm::vec2 targetDirection = -glm::normalize(glm::rotate(sampledCurveReferenceDirection, xyMovementRotAngle));
            float supportHeight = trData->getSupportHeight(initialFrame);
            initialPos = calcStrideStartingPoint(supportHeight, xyReferencePoint, xyDistanceToStart,
                targetDirection, 4, transformManager, staticMeshManager);
        }

        float targetDistance = glm::distance(baseCurve.getStart(), baseCurve.getEnd());
        glm::vec3 originalDirection = glm::normalize(baseCurve.getEnd() - baseCurve.getStart());
        glm::vec2 targetXYDirection = glm::normalize(glm::rotate(glm::vec2(originalDirection), xyMovementRotAngle));
        float supportHeightStart = trData->getSupportHeight(initialFrame);
        float supportHeightEnd = trData->getSupportHeight(finalFrame);
        std::vector<glm::vec3> strideData = calcStrideData(supportHeightStart, supportHeightEnd, 
            initialPos, targetDistance, targetXYDirection, 8, transformManager, staticMeshManager);
        if (strideData.size() == 0) { // si no es posible avanzar por la elevacion del terreno
            LIC<3> staticBaseCurve({ currentPos, currentPos }, { currentAnimTime, currentAnimTime + config->getAnimationDuration() });
            generateStaticTrajectory(EETrajectory(staticBaseCurve, TrajectoryType::STATIC, -2), ikChain, config, xyMovementRotAngle,
                transformManager, staticMeshManager);
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
        for (int i = 0; i < curvePointIndex_stridePointIndex.size(); i++){
            int curvePointIndex = curvePointIndex_stridePointIndex[i].first;
            int tgDataIndex = -1;
            for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
                if (curvePointIndex == m_tgData.pointIndexes[i]) {
                    tgDataIndex = i;
                    break;
                }
            }
            int stridePointIndex = curvePointIndex_stridePointIndex[i].second;
            // asignamos el valor minimo de z permitido al punto designado y a sus vecinos inmediatos
            if (0 < tgDataIndex) {
                m_tgData.minValues[(tgDataIndex - 1) * 3 + 2] = strideData[stridePointIndex][2];
            }
            m_tgData.minValues[tgDataIndex * 3 + 2] = strideData[stridePointIndex][2];
            if (tgDataIndex < (m_tgData.pointIndexes.size() - 1)) {
                m_tgData.minValues[(tgDataIndex + 1) * 3 + 2] = strideData[stridePointIndex][2];
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


        // testing 
		/*std::cout << "DEBUG DYNAMIC: " << std::endl;
        std::cout << "before gd " << std::endl;
		baseCurve.debugPrintCurvePoints();
        m_tgData.minValues = std::vector<float>(m_tgData.pointIndexes.size() * 3, std::numeric_limits<float>::lowest());
        initialArgs = std::vector<float>(initialArgs.size(), 0);
        for (int i = 1; i < baseCurve.getNumberOfPoints() - 1; i++) {
            baseCurve.setCurvePoint(i, glm::vec3(0));
        }*/
        //
              
        m_gradientDescent.setArgNum(initialArgs.size());
        m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, m_tgData.targetPosDelta, initialArgs);

		// testing
        //std::cout << "after gd " << std::endl;
		//baseCurve.debugPrintCurvePoints();

        float repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
        float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
        int currSubTrID = trData->getTargetTrajectory().getSubTrajectoryID();
        int newSubTrID = baseTrajectory.getSubTrajectoryID();
        if (currSubTrID == newSubTrID) {
			trData->setTargetTrajectory(LIC<3>::transition(trData->getTargetTrajectory().getEECurve(),
				baseCurve, transitionTime), TrajectoryType::DYNAMIC, baseTrajectory.getSubTrajectoryID());
        }
        else {
            trData->setTargetTrajectory(baseCurve, TrajectoryType::DYNAMIC, baseTrajectory.getSubTrajectoryID());
        }
		
    }

	

    void TrajectoryGenerator::generateHipTrajectory(IKRigConfig* config,
        float xyMovementRotAngle,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {

        bool allStatic = true;
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        for (int i = 0; i < m_ikChains.size(); i++) {
            ChainIndex ikChain = m_ikChains[i];
            if (config->getEETrajectoryData(ikChain)->getTargetTrajectory().isDynamic()) { 
                allStatic = false;
                break;
            }
        }

        HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
        
        if (allStatic) {
            float initialTime = config->getReproductionTime(currentFrame);
            float initialRotAngle = hipTrData->getSavedRotationAngle(currentFrame);
            glm::vec3 initialRotAxis = hipTrData->getSavedRotationAxis(currentFrame);
            glm::vec3 initialTrans = hipTrData->getSavedTranslation(currentFrame);
            glm::vec2 basePoint(initialTrans);
            // chequear si hay una curva previa generada
            if (!hipTrData->getTargetTranslations().inTRange(initialTime)) {
                initialTrans = glm::vec3(basePoint, calcHipAdjustedHeight(config, basePoint, initialTime, config->getAnimationTime(currentFrame)));
            }
            LIC<1> newHipRotAngles({ glm::vec1(initialRotAngle), glm::vec1(initialRotAngle) }, { initialTime, initialTime + config->getAnimationDuration() });
            LIC<3> newHipRotAxes({ initialRotAxis, initialRotAxis }, { initialTime, initialTime + config->getAnimationDuration() });
            LIC<3> newHipTrans({ initialTrans, initialTrans }, { initialTime, initialTime + config->getAnimationDuration() });
            hipTrData->setTargetRotationAngles(newHipRotAngles);
            hipTrData->setTargetRotationAxes(newHipRotAxes);
            hipTrData->setTargetTranslations(newHipTrans);
        }
        else {
            HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
            // buscamos la curva dinamica a la que le quede mas tiempo
            float tInfLimitRep = std::numeric_limits<float>::lowest();
            float tSupLimitRep = std::numeric_limits<float>::lowest();
            EETrajectory baseEETargetTr;
            EEGlobalTrajectoryData* baseEETrData;
            for (int i = 0; i < m_ikChains.size(); i++) {
                EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
                EETrajectory& currTr = trData->getTargetTrajectory();
                float tCurrentSupLimit = currTr.getEECurve().getTRange()[1];
                if (currTr.isDynamic() && tSupLimitRep < tCurrentSupLimit) {
                    tSupLimitRep = tCurrentSupLimit;
                    tInfLimitRep = currTr.getEECurve().getTRange()[0];
                    baseEETargetTr = currTr;
                    baseEETrData = trData;
                }
            }

            EETrajectory baseEEOriginalTr = baseEETrData->getSubTrajectoryByID(baseEETargetTr.getSubTrajectoryID());
            LIC<3>& baseEEOriginalCurve = baseEEOriginalTr.getEECurve();
            LIC<3>& baseEETargetCurve = baseEETargetTr.getEECurve();

			float tInfLimitExtendedAnim = baseEEOriginalCurve.getTRange()[0];
			float tSupLimitExtendedAnim = baseEEOriginalCurve.getTRange()[1];

            LIC<3> hipTrCurve = hipTrData->sampleOriginalTranslations(tInfLimitExtendedAnim, tSupLimitExtendedAnim);
            LIC<1> hipRotAnglCurve = hipTrData->sampleOriginalRotationAngles(tInfLimitExtendedAnim, tSupLimitExtendedAnim);
            LIC<3> hipRotAxCurve = hipTrData->sampleOriginalRotationAxes(tInfLimitExtendedAnim, tSupLimitExtendedAnim);
            
            // testing
            std::cout << "DEBUG HIP TR" << std::endl;
            std::cout << "original tr curve." << std::endl;
            hipTrCurve.debugPrintCurvePoints();

            float hipOriginalXYDistance = glm::length(glm::vec2(hipTrCurve.getEnd() - hipTrCurve.getStart()));

            // primer paso: calculo del punto inicial de la trayectoria
            float initialRotAngle = hipTrData->getSavedRotationAngle(currentFrame);
            glm::vec3 initialRotAxis = hipTrData->getSavedRotationAxis(currentFrame);
            glm::vec3 initialTrans = hipTrData->getSavedTranslation(currentFrame);
            // chequear si hay una curva previa generada
            if (!hipTrData->getTargetTranslations().inTRange(tInfLimitRep)) {
                initialTrans = glm::vec3(glm::vec2(initialTrans), 
                    calcHipAdjustedHeight(config, glm::vec2(initialTrans), tInfLimitRep, tInfLimitExtendedAnim));
            }
            
            // segundo paso: ajustar punto de maxima altura
            float hipHighOriginalTime_extendedAnim = baseEEOriginalTr.getHipMaxAltitudeTime();
            glm::vec3 hipHighOriginalPoint = baseEEOriginalCurve.evalCurve(hipHighOriginalTime_extendedAnim);
            float startHipHighOriginalDist = glm::distance(baseEEOriginalCurve.getStart(), hipHighOriginalPoint);
            float hipHighEndOriginalDist = glm::distance(hipHighOriginalPoint, baseEEOriginalCurve.getEnd());
            float startHipHighOriginalSpeed = startHipHighOriginalDist / (hipHighOriginalTime_extendedAnim - baseEEOriginalCurve.getTRange()[0]);
            float hipHighEndOriginalSpeed = hipHighEndOriginalDist / (baseEEOriginalCurve.getTRange()[1] - hipHighOriginalTime_extendedAnim);

            float hipHighOriginalTime_rep = baseEETargetCurve.getTRange()[0] + 
                (hipHighOriginalTime_extendedAnim - baseEEOriginalCurve.getTRange()[0]);
            glm::vec3 hipHighNewPoint_origTime = baseEETargetCurve.evalCurve(hipHighOriginalTime_rep);
            float startHighNewDist = glm::distance(baseEETargetCurve.getStart(), hipHighNewPoint_origTime);
            float highEndNewDist = glm::distance(hipHighNewPoint_origTime, baseEETargetCurve.getEnd());
            float startHighNewTime = startHighNewDist / startHipHighOriginalSpeed;
            float highEndNewTime = highEndNewDist / hipHighEndOriginalSpeed;

            float hipHighNewTimeFraction = funcUtils::getFraction(0, startHighNewTime + highEndNewTime, startHighNewTime);
            float hipHighNewTime_rep = funcUtils::lerp(baseEETargetCurve.getTRange()[0], baseEETargetCurve.getTRange()[1], hipHighNewTimeFraction);

            int hipHighPointIndex_hip = hipTrCurve.getClosestPointIndex(hipHighOriginalTime_extendedAnim);
			// ajuste a tiempo de reproduccion
			hipTrCurve.offsetTValues(-hipTrCurve.getTRange()[0] + tInfLimitRep);
			hipRotAnglCurve.offsetTValues(-hipRotAnglCurve.getTRange()[0] + tInfLimitRep);
			hipRotAxCurve.offsetTValues(-hipRotAxCurve.getTRange()[0] + tInfLimitRep);
            
			hipTrCurve.displacePointT(hipHighPointIndex_hip, 0, hipTrCurve.getNumberOfPoints() - 1, hipHighNewTime_rep, true);
			hipRotAnglCurve.displacePointT(hipHighPointIndex_hip, 0, hipTrCurve.getNumberOfPoints() - 1, hipHighNewTime_rep, true);
			hipRotAxCurve.displacePointT(hipHighPointIndex_hip, 0, hipTrCurve.getNumberOfPoints() - 1, hipHighNewTime_rep, false);

            // tercer paso: encontrar el punto final
            // creamos una curva que represente la trayectoria de caida
            glm::vec3 fall1 = hipTrCurve.getCurvePoint(hipHighPointIndex_hip);
            float fall1_time = hipTrCurve.getTValue(hipHighPointIndex_hip);
            glm::vec3 fall2(0);
            float fall2_time = 0;
            int pointNum = 0;
            for (int i = hipHighPointIndex_hip; i < hipTrCurve.getNumberOfPoints(); i++) {
                fall2 += hipTrCurve.getCurvePoint(i);
                fall2_time += hipTrCurve.getTValue(i);
                pointNum += 1;
            }
            fall2 /= pointNum;
            fall2_time /= pointNum;

            glm::vec3 fall3 = hipTrCurve.getEnd();
            float fall3_time = hipTrCurve.getTRange()[1];

            LIC<3> fallCurve({ fall1, fall2, fall3 }, { fall1_time, fall2_time, fall3_time });
            glm::vec3 fallAcc = fallCurve.getAcceleration(1);

            // calculamos el t aproximado del final de la caida
            float originalFallDuration = fallCurve.getTRange()[1] - fallCurve.getTRange()[0];
            float baseEEOriginalXYFallDist = glm::length(glm::vec2(baseEEOriginalCurve.getEnd() - 
                baseEEOriginalCurve.evalCurve(hipHighOriginalTime_extendedAnim)));
            float baseEETargetXYFallDist = glm::length(glm::vec2(baseEETargetCurve.getEnd() - baseEETargetCurve.evalCurve(hipHighNewTime_rep)));
            float xyFallDistRatio = baseEETargetXYFallDist / baseEEOriginalXYFallDist;
            float newFallDuration = originalFallDuration * xyFallDistRatio;

            // con el t encontrado y la aceleracion calculamos el punto final
            glm::vec3 finalTrans;
            // si el t cae antes de que termine la curva original
            float calcT = fall1_time + newFallDuration;
            if (calcT <= hipTrCurve.getTRange()[1]) {
                finalTrans = hipTrCurve.evalCurve(calcT);
            }
            else {// si no
                finalTrans = hipTrCurve.getEnd();
                glm::vec3 calcVel = hipTrCurve.getVelocity(fall1_time);
                int accSteps = 15;
                for (int i = 1; i <= accSteps; i++) {
                    finalTrans += calcVel * ((float)i / accSteps);
                    calcVel += fallAcc * newFallDuration * ((float)i / accSteps);
                }
            }
            std::vector<glm::vec3> newPoints;
            std::vector<float> newTimes;
            for (int i = 0; i <= hipHighPointIndex_hip; i++) {
				newPoints.push_back(hipTrCurve.getCurvePoint(i));
				newTimes.push_back(hipTrCurve.getTValue(i));
            }
            for (int i = hipHighPointIndex_hip + 1; i < hipTrCurve.getNumberOfPoints(); i++) {
                if (hipTrCurve.getTValue(i) < calcT) {
                    newPoints.push_back(hipTrCurve.getCurvePoint(i));
                    newTimes.push_back(hipTrCurve.getTValue(i));
                }
                else {
                    newPoints.push_back(finalTrans);
                    newTimes.push_back(calcT);
                    break;
                }
            }

            LIC<3> hipTrCurveAdjustedFall(newPoints, newTimes);
			// ajustamos los valores de t de la caida
            int finalIndex = hipTrCurveAdjustedFall.getNumberOfPoints() - 1;
            hipTrCurveAdjustedFall.displacePointT(finalIndex, hipHighPointIndex_hip, finalIndex, hipTrCurve.getTRange()[1], false);

            // actualizamos la orientacion de la curva y la trasladamos la curva al punto encontrado
            hipTrCurveAdjustedFall.translate(-hipTrCurveAdjustedFall.getStart());
            float baseEEOriginalXYDist = glm::length(glm::vec2(baseEEOriginalCurve.getEnd() - baseEEOriginalCurve.getStart()));
            float baseEETargetXYDist = glm::length(glm::vec2(baseEETargetCurve.getEnd() - baseEETargetCurve.getStart()));
            float eeXYDistRatio = baseEETargetXYDist / baseEEOriginalXYDist;
            float hipNewXYDist = glm::length(glm::vec2(hipTrCurveAdjustedFall.getEnd() - hipTrCurveAdjustedFall.getStart()));
            float hipXYDistRatio = hipNewXYDist / hipOriginalXYDistance;
            float eeHipXYDistRatio = eeXYDistRatio / hipXYDistRatio ;
            glm::vec3 scalingVec(eeHipXYDistRatio, eeHipXYDistRatio, 1);
            glm::fquat xyRot(xyMovementRotAngle, glm::vec3(0, 0, 1));
            hipTrCurveAdjustedFall.scale(scalingVec);            
            hipTrCurveAdjustedFall.rotate(xyRot);
            hipTrCurveAdjustedFall.translate(initialTrans);

            // falta modificar el valor de z del punto de maxima altura y luego aplicar descenso de gradiente
            LIC<3> hipTrFinalCurve = hipTrCurveAdjustedFall;

            glm::vec2 hipHighBasePoint = glm::vec2(hipTrFinalCurve.getCurvePoint(hipHighPointIndex_hip));
            glm::vec3 hipHighAdjustedZ = glm::vec3(hipHighBasePoint, calcHipAdjustedHeight(config, hipHighBasePoint, 
                hipHighNewTime_rep, hipHighOriginalTime_extendedAnim));
            hipTrFinalCurve.setCurvePoint(hipHighPointIndex_hip, hipHighAdjustedZ);

            m_tgData.pointIndexes.clear();
            m_tgData.pointIndexes.reserve(hipTrCurveAdjustedFall.getNumberOfPoints());
            m_tgData.minValues.clear();
            m_tgData.minValues.reserve(hipTrCurveAdjustedFall.getNumberOfPoints() * 3);
            for (int i = 1; i < hipTrCurveAdjustedFall.getNumberOfPoints() - 1; i++) {
                // Los extremos de la curva y el punto de max altura de la cadera se mantendran fijos.
                if (i != hipHighPointIndex_hip) {
                    m_tgData.pointIndexes.push_back(i);
                    for (int j = 0; j < 3; j++) {
                        m_tgData.minValues.push_back(std::numeric_limits<float>::lowest());
                    }
                }
                
                
            }

            // valores iniciales y curva base
            std::vector<float> initialArgs(m_tgData.pointIndexes.size() * 3);
            for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
                int pIndex = m_tgData.pointIndexes[i];
                for (int j = 0; j < 3; j++) {
                    initialArgs[i * 3 + j] = hipTrFinalCurve.getCurvePoint(pIndex)[j];
                }
            }
            

			// testing
            std::cout << "before gr descent adj fall." << std::endl;
            hipTrCurveAdjustedFall.debugPrintCurvePoints();
			std::cout << "before gr descent final curve." << std::endl;
			hipTrFinalCurve.debugPrintCurvePoints();

            m_tgData.baseCurve = hipTrCurveAdjustedFall;
            m_tgData.varCurve = &hipTrFinalCurve;
            m_gradientDescent.setArgNum(initialArgs.size());
            m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, m_tgData.targetPosDelta, initialArgs);

			// testing
            std::cout << "after gr descent." << std::endl;
			hipTrFinalCurve.debugPrintCurvePoints();

			float repCountOffset = config->getNextFrameIndex() == 0 ? 1 : 0;
			float transitionTime = config->getReproductionTime(config->getNextFrameIndex(), repCountOffset);
            if (hipTrData->getTargetTranslations().inTRange(transitionTime)) {
                hipTrData->setTargetTranslations(LIC<3>::transition(hipTrData->getTargetTranslations(), hipTrFinalCurve, transitionTime));
                hipTrData->setTargetRotationAngles(LIC<1>::transition(hipTrData->getTargetRotationAngles(), hipRotAnglCurve, transitionTime));
                hipTrData->setTargetRotationAxes(LIC<3>::transition(hipTrData->getTargetRotationAxes(), hipRotAxCurve, transitionTime));
            }
            else {
                hipTrData->setTargetRotationAngles(hipRotAnglCurve);
                hipTrData->setTargetRotationAxes(hipRotAxCurve);
                hipTrData->setTargetTranslations(hipTrFinalCurve);
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
			glm::vec2 testPoint = xyReferencePoint + targetDirection * targetDistance * ((float)i / stepNum);
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
		for (int i = 0; i < collectedPoints.size(); i++) {
			float distance = glm::distance(startingPoint, collectedPoints[i]);
			if (distance <= targetDistance+epsilon && previousDistance * 0.9 <= distance) {
				strideDataPoints.push_back(collectedPoints[i]);
				previousDistance = distance;
			}
			else { break; }
		}
		return strideDataPoints;
	}

	float TrajectoryGenerator::calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float targetCurvesTime_rep,
		float originalCurvesTime_extendedAnim) {
		HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();
		float originalCurvesTime_anim = config->adjustAnimationTime(originalCurvesTime_extendedAnim);
		// distancias originales de ee's con cadera
		std::vector<float> origDistances(m_ikChains.size());
		LIC<3> hipTrCurve = hipTrData->sampleOriginalTranslations(originalCurvesTime_extendedAnim,	originalCurvesTime_extendedAnim + config->getAnimationDuration());
        glm::vec3 hipPoint = hipTrCurve.evalCurve(originalCurvesTime_extendedAnim);
		for (int i = 0; i < m_ikChains.size(); i++) {
			EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
			glm::vec3 eePoint = trData->getSubTrajectory(originalCurvesTime_anim).getEECurve().evalCurve(originalCurvesTime_anim);
			origDistances[i] = glm::distance(hipPoint, eePoint);
		}

		std::vector<glm::vec3> targetPoints(m_ikChains.size());
		float zValue = std::numeric_limits<float>::lowest();
		for (int i = 0; i < m_ikChains.size(); i++) {
			EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
			LIC<3> eeCurve = trData->getTargetTrajectory().getEECurve();
			targetPoints[i] = eeCurve.inTRange(targetCurvesTime_rep) ? eeCurve.evalCurve(targetCurvesTime_rep) :
				eeCurve.getCurvePoint(eeCurve.getClosestPointIndex(targetCurvesTime_rep));
			if (zValue < targetPoints[i][2]) { zValue = targetPoints[i][2]; }
		}

		// valor mas bajo de z para las tr actuales de los ee
		zValue += m_ikRig->getRigHeight()*0.8;
		float zDelta = m_ikRig->getRigHeight() / 500;
		std::vector<bool> conditions1(m_ikChains.size(), false);
        std::vector<bool> conditions2(m_ikChains.size(), true);
		int maxSteps = 1000;
		int steps = 0;
        std::vector<float> prevDistances(m_ikChains.size(), std::numeric_limits<float>::max());
		while (!funcUtils::conditionVector_AND(conditions1) && funcUtils::conditionVector_OR(conditions2) && steps < maxSteps) {
			zValue -= zDelta;
            std::vector<float> currDistances(m_ikChains.size());
			for (int i = 0; i < m_ikChains.size(); i++) {
				currDistances[i] = glm::distance(targetPoints[i], glm::vec3(basePoint, zValue));
				conditions1[i] = currDistances[i] <= origDistances[i];
                conditions2[i] = currDistances[i] <= prevDistances[i];
			}
            prevDistances = currDistances;

			steps += 1;
		}

		return zValue;
	}


    
}