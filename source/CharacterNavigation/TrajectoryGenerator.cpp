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
            glm::vec3 baseLVel = dataPtr->baseVelocitiesL[i];
            glm::vec3 baseRVel = dataPtr->baseVelocitiesR[i];
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
		glm::vec3 rVel = dataPtr->varCurve->getVelocity(t_kCurr, true);
		glm::vec3 baseLVel = dataPtr->baseVelocitiesL[varIndex / D];
		glm::vec3 baseRVel = dataPtr->baseVelocitiesR[varIndex / D];
        float result = 0;
        result += 2 * (lVel[coordIndex] / glm::length(lVel) - baseLVel[coordIndex] / glm::length(baseLVel))
            * ((1 / (t_kCurr - t_kPrev)) * glm::length(lVel) - lVel[coordIndex] * (lVel[coordIndex]/glm::length(lVel)) * (1 / (t_kCurr - t_kPrev)))/
            glm::length2(lVel);
        result += 2 * (rVel[coordIndex] / glm::length(rVel) - baseRVel[coordIndex] / glm::length(baseRVel))
            * ((-1 / (t_kNext - t_kCurr)) * glm::length(rVel) - rVel[coordIndex] * (rVel[coordIndex] / glm::length(rVel)) * (-1 / (t_kNext - t_kCurr))) /
            glm::length2(rVel);
		return dataPtr->alphaValue * result;
	};

    // segundo termino: acercar los modulos de las velocidades
    std::function<float(const std::vector<float>&, TGData*)> term2Function =
        [](const std::vector<float>& varPCoord, TGData* dataPtr)->float {
        float result = 0;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            int pIndex = dataPtr->pointIndexes[i];
            float tVal = dataPtr->varCurve->getTValue(pIndex);
            result += glm::distance2(dataPtr->varCurve->getVelocity(tVal),  dataPtr->baseVelocitiesL[i]);
            result += glm::distance2(dataPtr->varCurve->getVelocity(tVal, true), dataPtr->baseVelocitiesR[i]);
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
		glm::vec3 baseLVel = dataPtr->baseVelocitiesL[varIndex / D];
		glm::vec3 baseRVel = dataPtr->baseVelocitiesR[varIndex / D];
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
                    argsRawDelta[i * D + j] = 0;
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
        m_tgData.descentRate = 0.001;
        m_tgData.maxIterations = 200;
        m_tgData.alphaValue = 0.8;
        m_tgData.betaValue = 1.0;
        m_tgData.targetPosDelta = m_ikRig->getRigHeight()/pow(10, 8);
    }

	void TrajectoryGenerator::generateNewTrajectories(AnimationIndex animIndex,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		// las trayectorias anteriores siempre deben llegar hasta el currentFrame

		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		float xyRotationAngle = glm::orientedAngle(glm::vec3(config->getHipTrajectoryData()->getOriginalFrontVector(), 0),
			glm::vec3(m_ikRig->getFrontVector(), 0), glm::vec3(0, 0, 1));

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

		// testing
		baseCurve.debugPrintCurvePoints();

        baseCurve.fitEnds(initialPos, finalPos);

		// testing
		baseCurve.debugPrintCurvePoints();

        // setear los minimos de altura y aplicar el descenso de gradiente
        m_tgData.pointIndexes.clear();
        m_tgData.pointIndexes.reserve(baseCurve.getNumberOfPoints());
        m_tgData.baseVelocitiesR.clear();
        m_tgData.baseVelocitiesR.reserve(baseCurve.getNumberOfPoints());
		m_tgData.baseVelocitiesL.clear();
		m_tgData.baseVelocitiesL.reserve(baseCurve.getNumberOfPoints());
        m_tgData.minValues.clear();
        m_tgData.minValues.reserve(baseCurve.getNumberOfPoints() * 3);
        std::vector<std::pair<int, int>> curvePointIndex_stridePointIndex;
        std::vector<int> baseCurveIndices;
        std::vector<int> strideDataIndices;
        for (int i = 1; i < baseCurve.getNumberOfPoints() - 1; i++) { baseCurveIndices.push_back(i); }
        for (int i = 0; i < strideDataIndices.size(); i++) { strideDataIndices.push_back(i); }
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

        for (int i = 1; i < baseCurve.getNumberOfPoints() - 1; i++) {
            m_tgData.pointIndexes.push_back(i);
        }

        for (int i = 0; i < m_tgData.pointIndexes.size()*3;i++) {
            m_tgData.minValues.push_back(std::numeric_limits<float>::lowest());
        }
        for (int i = 0; i < curvePointIndex_stridePointIndex.size(); i++){
            int curvePointIndex = curvePointIndex_stridePointIndex[i].first;
            int stridePointIndex = curvePointIndex_stridePointIndex[i].second;
            // asignamos el valor minimo de z permitido al punto designado y a sus vecinos inmediatos
            if (0 < curvePointIndex) {
                m_tgData.minValues[(curvePointIndex - 1) * 3 + 2] = strideData[stridePointIndex][2];
            }
            m_tgData.minValues[(curvePointIndex) * 3 + 2] = strideData[stridePointIndex][2];
            if (curvePointIndex < (m_tgData.pointIndexes.size() - 1)) {
                m_tgData.minValues[(curvePointIndex + 1) * 3 + 2] = strideData[stridePointIndex][2];
            }
        }
        // valores iniciales y velocidades base
        std::vector<float> initialArgs(m_tgData.pointIndexes.size() * 3);
        for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
            int pIndex = m_tgData.pointIndexes[i];
            for (int j = 0; j < 3; j++) {
                initialArgs[i * 3 + j] = baseCurve.getCurvePoint(pIndex)[j];
            }
            m_tgData.baseVelocitiesL.push_back(baseCurve.getVelocity(baseCurve.getTValue(pIndex)));
            m_tgData.baseVelocitiesR.push_back(baseCurve.getVelocity(baseCurve.getTValue(pIndex), true));
        }

        // seteo de otros parametros
        m_tgData.varCurve = &baseCurve;
        m_gradientDescent.setArgNum(initialArgs.size());

        // testing
        m_tgData.minValues = std::vector<float>(m_tgData.pointIndexes.size() * 3, std::numeric_limits<float>::lowest());
        initialArgs = std::vector<float>(initialArgs.size(), 0);
        for (int i = 1; i < baseCurve.getNumberOfPoints() - 1; i++) {
            baseCurve.setCurvePoint(i, glm::vec3(0));
        }


        m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, m_tgData.targetPosDelta, initialArgs);

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

        // testing
        auto targetCurve = trData->getTargetTrajectory().getEECurve();
        targetCurve.debugPrintCurvePoints();
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
            float hipHighNewTime = funcUtils::lerp(baseEETargetCurve.getTRange()[0], baseEETargetCurve.getTRange()[1], hipHighNewTimeFraction);

            int hipHighPointIndex_hip = hipTrCurve.getClosestPointIndex(hipHighOriginalTime_extendedAnim);
			// ajuste a tiempo de reproduccion
			hipTrCurve.offsetTValues(-hipTrCurve.getTRange()[0] + tInfLimitRep);
			hipRotAnglCurve.offsetTValues(-hipRotAnglCurve.getTRange()[0] + tInfLimitRep);
			hipRotAxCurve.offsetTValues(-hipRotAxCurve.getTRange()[0] + tInfLimitRep);
            
			hipTrCurve.displacePointT(hipHighPointIndex_hip, 0, hipTrCurve.getNumberOfPoints() - 1, hipHighNewTime, true);
			hipRotAnglCurve.displacePointT(hipHighPointIndex_hip, 0, hipTrCurve.getNumberOfPoints() - 1, hipHighNewTime, true);
			hipRotAxCurve.displacePointT(hipHighPointIndex_hip, 0, hipTrCurve.getNumberOfPoints() - 1, hipHighNewTime, false);

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
            float baseEETargetXYFallDist = glm::length(glm::vec2(baseEETargetCurve.getEnd() - baseEETargetCurve.evalCurve(hipHighNewTime)));
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
            hipTrCurveAdjustedFall.translate(-hipTrCurveAdjustedFall.getStart()); // llevar al origen
            // escalamos la trayectoria para ajustarla a la distancia recorrida
            float baseEEOriginalXYDist = glm::length(glm::vec2(baseEEOriginalCurve.getEnd() - baseEEOriginalCurve.getStart()));
            float baseEETargetXYDist = glm::length(glm::vec2(baseEETargetCurve.getEnd() - baseEETargetCurve.getStart()));
            float eeXYDistRatio = baseEETargetXYDist / baseEEOriginalXYDist;
            float hipNewXYDist = glm::length(glm::vec2(hipTrCurveAdjustedFall.getEnd() - hipTrCurveAdjustedFall.getStart()));
            float hipXYDistRatio = hipNewXYDist / hipOriginalXYDistance;
            float hipEEXYDistRatio = hipXYDistRatio / eeXYDistRatio;
            glm::vec3 scalingVec(glm::vec2((hipTrCurveAdjustedFall.getEnd() - hipTrCurveAdjustedFall.getStart()) * (1 / hipEEXYDistRatio)), 0);
            hipTrCurveAdjustedFall.scale(scalingVec);
            glm::fquat xyRot(xyMovementRotAngle, glm::vec3(0, 0, 1));
            hipTrCurveAdjustedFall.rotate(xyRot);
            hipTrCurveAdjustedFall.translate(initialTrans);

            // falta modificar el valor de z del punto de maxima altura y luego aplicar descenso de gradiente
            LIC<3> hipTrFinalCurve = hipTrCurveAdjustedFall;

            glm::vec2 hipHighBasePoint = glm::vec2(hipTrFinalCurve.getCurvePoint(hipHighPointIndex_hip));
            glm::vec3 hipHighAdjustedZ = glm::vec3(hipHighBasePoint, calcHipAdjustedHeight(config, hipHighBasePoint, 
                hipHighNewTime, config->adjustAnimationTime(hipHighOriginalTime_extendedAnim)));
            hipTrFinalCurve.setCurvePoint(hipHighPointIndex_hip, hipHighAdjustedZ);

            m_tgData.pointIndexes.clear();
            m_tgData.pointIndexes.reserve(hipTrCurveAdjustedFall.getNumberOfPoints());
            m_tgData.baseVelocitiesL.clear();
            m_tgData.baseVelocitiesL.reserve(hipTrCurveAdjustedFall.getNumberOfPoints());
			m_tgData.baseVelocitiesR.clear();
			m_tgData.baseVelocitiesR.reserve(hipTrCurveAdjustedFall.getNumberOfPoints());
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

            // valores iniciales y velocidades base
            std::vector<float> initialArgs(m_tgData.pointIndexes.size() * 3);
            for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
                int pIndex = m_tgData.pointIndexes[i];
                for (int j = 0; j < 2; j++) {
                    initialArgs[i * 3 + j] = hipTrCurveAdjustedFall.getCurvePoint(pIndex)[j];
                }
                m_tgData.baseVelocitiesL.push_back(hipTrCurveAdjustedFall.getVelocity(hipTrCurveAdjustedFall.getTValue(pIndex)));
                m_tgData.baseVelocitiesR.push_back(hipTrCurveAdjustedFall.getVelocity(hipTrCurveAdjustedFall.getTValue(pIndex), true));
            }

            // seteo de otros parametros
            m_tgData.varCurve = &hipTrFinalCurve;
            m_gradientDescent.setArgNum(initialArgs.size());
            m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, m_tgData.targetPosDelta, initialArgs);

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
		LIC<3> hipTrCurve = hipTrData->sampleOriginalTranslations(originalCurvesTime_extendedAnim,
			originalCurvesTime_extendedAnim + config->getAnimationDuration());
		for (int i = 0; i < m_ikChains.size(); i++) {
			EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
			glm::vec3 hipPoint = hipTrCurve.evalCurve(originalCurvesTime_extendedAnim);
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
		zValue += m_ikRig->getRigHeight() / 1.5;
		float zDelta = m_ikRig->getRigHeight() / 1000;
		std::vector<bool> conditions(m_ikChains.size());
		for (int i = 0; i < m_ikChains.size(); i++) { conditions[i] = false; }
		int maxSteps = 5000;
		int steps = 0;
		while (!funcUtils::conditionArray_AND(conditions)) {
			zValue -= zDelta;
			// update conditions
			for (int i = 0; i < m_ikChains.size(); i++) {
				float currentDistance = glm::distance(targetPoints[i], glm::vec3(basePoint, zValue));
				conditions[i] = currentDistance <= origDistances[i];
			}
			steps += 1;
			if (maxSteps < steps) {
				break;
			}
		}

		return zValue;
	}


    
}