#include "TrajectoryGenerator.hpp"
#include "../Core/GlmUtils.hpp"
#include "../Core/FuncUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/vector_angle.hpp"
#include "IKRig.hpp"

namespace Mona{

    EETrajectory::EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType) {
        m_curve = trajectory;
        m_trajectoryType = trajectoryType;
    }

    EETrajectory EEGlobalTrajectoryData::getSubTrajectory(float animationTime) {
        for (int i = 0; i < m_originalSubTrajectories.size(); i++) {
            if (m_originalSubTrajectories[i].getEECurve().inOpenRightTRange(animationTime)) {
                return m_originalSubTrajectories[i];
            }
        }
        MONA_LOG_ERROR("EETrajectoryData: AnimationTime was not valid.");
        return EETrajectory();
    }

    // funciones para descenso de gradiente
    std::function<float(const std::vector<float>&, TGData*)> term1Function =
        [](const std::vector<float>& varPCoord, TGData* dataPtr)->float {
        float result = 0;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            int pIndex = dataPtr->pointIndexes[i];
            float tVal = dataPtr->varCurve->getTValue(pIndex);
            result += glm::distance2(dataPtr->varCurve->getVelocity(tVal),
                dataPtr->baseVelocities[i]);
        }
        return result;
    };

    std::function<float(const std::vector<float>&, int, TGData*)> term1PartialDerivativeFunction =
        [](const std::vector<float>& varPCoord, int varIndex, TGData* dataPtr)->float {
        int D = 3;
        int pIndex = dataPtr->pointIndexes[varIndex / D];
        int coordIndex = varIndex % D;
        float tVal = dataPtr->varCurve->getTValue(pIndex);
        float result = pIndex != 0 ?
            2 * (dataPtr->varCurve->getVelocity(tVal)[coordIndex] - dataPtr->baseVelocities[varIndex / D][coordIndex])
            / (dataPtr->varCurve->getTValue(pIndex) - dataPtr->varCurve->getTValue(pIndex - 1)) : 0;

        return result;
    };

    std::function<void(std::vector<float>&, TGData*)>  postDescentStepCustomBehaviour = 
        [](std::vector<float>& varPCoord, TGData* dataPtr)->void {
        glm::vec3 newPos;
        int D = 3;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            for (int j = 0; j < D; j++) {
                newPos[j] = std::max(dataPtr->minValues[i * D + j], varPCoord[i * D + j]);
            }
            int pIndex = dataPtr->pointIndexes[i];
            dataPtr->varCurve->setCurvePoint(pIndex, newPos);
        }
    };


    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains) {
        m_ikRig = ikRig;
        m_ikChains = ikChains;

        // descenso para posiciones (dim 3)
        FunctionTerm<TGData> term(term1Function, term1PartialDerivativeFunction);
        m_gradientDescent = GradientDescent<TGData>({ term }, 0, &m_tgData, postDescentStepCustomBehaviour);
    }

    glm::vec3 TrajectoryGenerator::calcStrideStartingPoint(float supportHeight,
        glm::vec3 referencePoint, float targetDistance, 
        glm::vec2 targetDirection, int stepNum,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        glm::vec2 refPointBase(referencePoint[0], referencePoint[1]);
        std::vector<glm::vec3> collectedPoints;
        collectedPoints.reserve(stepNum);
        for (int i = 1; i <= stepNum; i++) {
            glm::vec2 testPoint = refPointBase + targetDirection * targetDistance * ((float)i / stepNum);
            float calcHeight = m_environmentData.getTerrainHeight(testPoint,
                transformManager, staticMeshManager);
            collectedPoints.push_back(glm::vec3(testPoint, supportHeight + calcHeight));
        }
        float minDistanceDiff = std::numeric_limits<float>::max();
        glm::vec3 closest = collectedPoints[0];
        for (int i = 0; i < collectedPoints.size(); i++) {
            float distDiff = std::abs(glm::distance(referencePoint, collectedPoints[i]) - targetDistance);
            if ( distDiff < minDistanceDiff) {
                minDistanceDiff = distDiff;
                closest = collectedPoints[i];
            }
        }
        return closest;
    }

    std::vector<glm::vec3> TrajectoryGenerator::calcStrideData(float supportHeight, glm::vec3 startingPoint, float targetDistance,
        glm::vec2 targetDirection, int stepNum,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        std::vector<glm::vec3> collectedPoints;
        collectedPoints.reserve(stepNum);
        for (int i = 1; i <= stepNum; i++) {
            glm::vec2 testPoint = glm::vec2(startingPoint) + targetDirection * targetDistance * ((float)i / stepNum);
            float calcHeight = supportHeight + m_environmentData.getTerrainHeight(testPoint, transformManager, staticMeshManager);
            collectedPoints.push_back(glm::vec3(testPoint, calcHeight));
        }
        std::vector<glm::vec3> strideDataPoints;
        strideDataPoints.reserve(stepNum);
        float previousDistance = 0;
        for (int i = 0; i < collectedPoints.size(); i++) {
            float distance = glm::distance(startingPoint, collectedPoints[i]);
            if ( distance <= targetDistance &&  previousDistance*0.9 <= distance) {
                strideDataPoints.push_back(collectedPoints[i]);
                previousDistance = distance;
            }
            else { break; }
        }
        return strideDataPoints;
    }

    void TrajectoryGenerator::generateStaticTrajectory(EETrajectory baseTrajectory,
        ChainIndex ikChain, IKRigConfig* config,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);

        LIC<3>& baseCurve = baseTrajectory.getEECurve();
        FrameIndex initialFrame = config->getCurrentFrameIndex();
        float initialTime = config->getReproductionTime(initialFrame);
        glm::vec3 initialPos = trData->getSavedPosition(initialFrame);
        // chequear si hay una curva previa generada
        if (!trData->getTargetTrajectory().getEECurve().inTRange(initialTime)) {
            float calcHeight = m_environmentData.getTerrainHeight(glm::vec2(initialPos), transformManager, staticMeshManager);
            initialPos = glm::vec3(glm::vec2(initialPos), trData->getSupportHeight(initialFrame) + calcHeight);
        }
        // correccion de posicion y llevar a reproduction time
        baseCurve.translate(-baseCurve.getStart());
        baseCurve.translate(initialPos);
        baseCurve.offsetTValues(-baseCurve.getTValue(0));
        baseCurve.offsetTValues(initialTime);
        trData->setTargetTrajectory(baseCurve, TrajectoryType::STATIC);
    }

    void TrajectoryGenerator::generateDynamicTrajectory(EETrajectory baseTrajectory, 
        ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {

        LIC<3>& baseCurve = baseTrajectory.getEECurve();
        EEGlobalTrajectoryData* trData = config->getEETrajectoryData(ikChain);
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        FrameIndex initialFrame = config->getFrame(baseCurve.getTValue(0));

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
            float floorDistanceToStart = glm::distance(glm::vec2(baseCurve.getStart()), glm::vec2(sampledCurveReferencePoint));
            glm::vec3 floorReferencePoint(glm::vec2(currentPos), m_environmentData.getTerrainHeight(glm::vec2(currentPos),
                transformManager, staticMeshManager));
            glm::vec2 sampledCurveReferenceDirection = glm::vec2(sampledCurveReferencePoint) - glm::vec2(baseCurve.getStart());
            glm::vec2 targetDirection = -glm::normalize(glm::rotate(sampledCurveReferenceDirection, xyMovementRotAngle));
            float supportHeight = trData->getSupportHeight(initialFrame);
            initialPos = calcStrideStartingPoint(supportHeight, floorReferencePoint, floorDistanceToStart,
                targetDirection, 4, transformManager, staticMeshManager);
        }

        float targetDistance = glm::distance(baseCurve.getStart(), baseCurve.getEnd());
        glm::vec3 originalDirection = glm::normalize(baseCurve.getEnd() - baseCurve.getStart());
        glm::vec2 targetXYDirection = glm::rotate(glm::vec2(originalDirection), xyMovementRotAngle);
        float supportHeight = trData->getSupportHeight(initialFrame);
        std::vector<glm::vec3> strideData = calcStrideData(supportHeight, initialPos, targetDistance, targetXYDirection, 8, transformManager, staticMeshManager);
        if (strideData.size() == 0) { // si no es posible avanzar por la elevacion del terreno
            LIC<3> staticBaseCurve({ currentPos, currentPos }, { currentAnimTime, currentAnimTime + config->getAnimationDuration() });
            generateStaticTrajectory(EETrajectory(staticBaseCurve, TrajectoryType::STATIC), 
                ikChain, config, transformManager, staticMeshManager);
            return;
        }
        glm::vec3 finalPos = strideData.back();
        
        // mover la curva al origen
        baseCurve.translate(-baseCurve.getStart());
        // rotarla la subtrayectoria para que quede en linea con las pos inicial y final
        glm::fquat targetRotation = glm::identity<glm::fquat>();
        glm::vec3 targetDirection = glm::normalize(finalPos - initialPos);
        if (originalDirection != targetDirection) {
            glm::vec3 rotAxis = glm::normalize(glm::cross(originalDirection, targetDirection));
            float rotAngle = glm::orientedAngle(originalDirection, targetDirection, rotAxis);
            targetRotation = glm::fquat(rotAngle, rotAxis);
        }
        baseCurve.rotate(targetRotation);        

        // escalarla y moverla para que calce con las posiciones
        float origLength = glm::distance(baseCurve.getStart(), baseCurve.getEnd()); // actualmente parte en el origen
        float targetLength = glm::distance(initialPos, finalPos);
        baseCurve.scale(glm::vec3(targetLength/origLength));
        baseCurve.translate(initialPos);

        // setear los minimos de altura y aplicar el descenso de gradiente
        m_tgData.pointIndexes.clear();
        m_tgData.pointIndexes.reserve(baseCurve.getNumberOfPoints());
        m_tgData.baseVelocities.clear();
        m_tgData.baseVelocities.reserve(baseCurve.getNumberOfPoints());
        m_tgData.minValues.clear();
        m_tgData.minValues.reserve(baseCurve.getNumberOfPoints() * 3);
        int stridePointInd = 0;
        float stridePointPreviousDist = glm::distance2(glm::vec2(baseCurve.getCurvePoint(0)), glm::vec2(strideData[stridePointInd]));
        std::vector<std::pair<int, int>> curvePointIndex_stridePointIndex;
        for (int i = 1; i < baseCurve.getNumberOfPoints(); i++) {
            // Los extremos de la curva se mantendran fijos.
            m_tgData.pointIndexes.push_back(i);
            float currDist = glm::distance2(glm::vec2(baseCurve.getCurvePoint(i)), glm::vec2(strideData[stridePointInd]));
            if (stridePointPreviousDist < currDist) {
                // si la distancia del punto actual es mayor, entonces el anterior se escoge como el mas cercano
                curvePointIndex_stridePointIndex.push_back({i-1, stridePointInd});
                stridePointInd += 1;
                if (stridePointInd == strideData.size()) { break; }
            }
            else if (i == baseCurve.getNumberOfPoints() - 1) {
                curvePointIndex_stridePointIndex.push_back({ i, stridePointInd });
            }
        }
        for (int i = 0; i < m_tgData.pointIndexes.size()*3;i++) {
            m_tgData.minValues.push_back(std::numeric_limits<float>::min());
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
            for (int j = 0; j < 2; j++) {
                initialArgs[i * 3 + j] = baseCurve.getCurvePoint(pIndex)[j];
            }
            m_tgData.baseVelocities.push_back(baseCurve.getVelocity(baseCurve.getTValue(pIndex)));
        }

        // seteo de otros parametros (TODO)
        m_tgData.descentRate;
        m_tgData.maxIterations = 100;
        m_tgData.varCurve = &baseCurve;
        m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, initialArgs);

        float transitionTime = config->getCurrentReproductionTime();
        if (trData->getTargetTrajectory().getEECurve().inTRange(transitionTime)) {
            trData->setTargetTrajectory(LIC<3>::transition(trData->getTargetTrajectory().getEECurve(), 
                baseCurve, transitionTime), TrajectoryType::DYNAMIC);
        }
        else {
            trData->setTargetTrajectory(baseCurve, TrajectoryType::DYNAMIC);
        }   
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
            generateStaticTrajectory(EETrajectory(baseCurve, TrajectoryType::STATIC), ikChain, config, transformManager, staticMeshManager);
            return;
        }
        EETrajectory originalTrajectory = trData->getSubTrajectory(currentAnimTime);
        if (originalTrajectory.isDynamic()) {
            generateDynamicTrajectory(originalTrajectory, ikChain, config, xyMovementRotAngle, transformManager, staticMeshManager);
        }
        else {
            generateStaticTrajectory(originalTrajectory, ikChain, config, transformManager, staticMeshManager);
        }    
    }

    float TrajectoryGenerator::calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float targetCurvesTime_rep, 
        float originalCurvesTime_anim) {
        HipGlobalTrajectoryData* hipTrData = config->getHipTrajectoryData();

        // distancias originales de ee's con cadera
        std::vector<float> origDistances(m_ikChains.size());
        for (int i = 0; i < m_ikChains.size(); i++) {
            EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
            glm::vec3 hipPoint = hipTrData->getOriginalTranslations().evalCurve(originalCurvesTime_anim);
            glm::vec3 eePoint = trData->getSubTrajectory(originalCurvesTime_anim).getEECurve().evalCurve(originalCurvesTime_anim);
            origDistances[i] = glm::distance(hipPoint, eePoint);
        }

        std::vector<glm::vec3> targetPoints(m_ikChains.size());
        float zValue = std::numeric_limits<float>::min();
        for (int i = 0; i < m_ikChains.size(); i++) {
            EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
            targetPoints[i] = trData->getTargetTrajectory().getEECurve().evalCurve(targetCurvesTime_rep);
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

    void TrajectoryGenerator::generateHipTrajectory(IKRigConfig* config,
        float xyMovementRotAngle,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {

        bool allStatic = true;
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        for (int i = 0; i < m_ikChains.size(); i++) {
            ChainIndex ikChain = m_ikChains[i];
            if (config->getEETrajectoryData(ikChain)->getSubTrajectory(config->getAnimationTime(currentFrame)).isDynamic()) { 
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
            float tInfLimitRep = std::numeric_limits<float>::min();
            float tSupLimitRep = std::numeric_limits<float>::min();
            EETrajectory baseEETargetTr;
            EEGlobalTrajectoryData* baseEETargetTrData;
            for (int i = 0; i < m_ikChains.size(); i++) {
                EEGlobalTrajectoryData* trData = config->getEETrajectoryData(m_ikChains[i]);
                EETrajectory& currTr = trData->getTargetTrajectory();
                float tCurrentSupLimit = currTr.getEECurve().getTRange()[1];
                if (currTr.isDynamic() && tSupLimitRep < tCurrentSupLimit) {
                    tSupLimitRep = tCurrentSupLimit;
                    tInfLimitRep = currTr.getEECurve().getTRange()[0];
                    baseEETargetTr = currTr;
                    baseEETargetTrData = trData;
                }
            }
            float tInfLimitAnim = config->getAnimationTime(tInfLimitRep);
            float tSupLimitAnim = config->getAnimationTime(tSupLimitRep);

            EETrajectory baseEEOriginalTr = baseEETargetTrData->getSubTrajectory(tInfLimitAnim);
            LIC<3>& baseEEOriginalCurve = baseEEOriginalTr.getEECurve();
            LIC<3>& baseEETargetCurve = baseEETargetTr.getEECurve();

            LIC<3> hipTrCurve = hipTrData->getOriginalTranslations().sample(tInfLimitAnim, tSupLimitAnim);
            LIC<1> hipRotAnglCurve = hipTrData->getOriginalRotationAngles().sample(tInfLimitAnim, tSupLimitAnim);
            LIC<3> hipRotAxCurve = hipTrData->getOriginalRotationAxes().sample(tInfLimitAnim, tSupLimitAnim);

            // ajuste a tiempo de reproduccion
            hipTrCurve.offsetTValues(-tInfLimitAnim + tInfLimitRep);
            hipRotAnglCurve.offsetTValues(-tInfLimitAnim + tInfLimitRep);
            hipRotAxCurve.offsetTValues(-tInfLimitAnim + tInfLimitRep);

            float hipOriginalXYDistance = glm::length(glm::vec2(hipTrCurve.getEnd() - hipTrCurve.getStart()));

            // primer paso: calculo del punto inicial de la trayectoria
            float initialRotAngle = hipTrData->getSavedRotationAngle(currentFrame);
            glm::vec3 initialRotAxis = hipTrData->getSavedRotationAxis(currentFrame);
            glm::vec3 initialTrans = hipTrData->getSavedTranslation(currentFrame);
            // chequear si hay una curva previa generada
            if (!hipTrData->getTargetTranslations().inTRange(tInfLimitRep)) {
                initialTrans = glm::vec3(glm::vec2(initialTrans), calcHipAdjustedHeight(config, glm::vec2(initialTrans), tInfLimitRep, tInfLimitAnim));
            }
            
            // segundo paso: ajustar punto de maxima altura
            float hipHighOriginalTime = baseEEOriginalCurve.getTValue(baseEEOriginalTr.getHipMaxAltitudeIndex());
            glm::vec3 hipHighOriginalPoint = baseEEOriginalCurve.evalCurve(hipHighOriginalTime);
            float startHipHighOriginalDist = glm::distance(baseEEOriginalCurve.getStart(), hipHighOriginalPoint);
            float hipHighEndOriginalDist = glm::distance(hipHighOriginalPoint, baseEEOriginalCurve.getEnd());
            float startHipHighOriginalSpeed = startHipHighOriginalDist / (hipHighOriginalTime - baseEEOriginalCurve.getTRange()[0]);
            float hipHighEndOriginalSpeed = hipHighEndOriginalDist / (baseEEOriginalCurve.getTRange()[1] - hipHighOriginalTime);
            glm::vec3 hipHighNewPoint_origTime = baseEETargetCurve.evalCurve(hipHighOriginalTime);
            float startHighNewDist = glm::distance(baseEETargetCurve.getStart(), hipHighNewPoint_origTime);
            float highEndNewDist = glm::distance(hipHighNewPoint_origTime, baseEETargetCurve.getEnd());
            float startHighNewTime = startHighNewDist / startHipHighOriginalSpeed;
            float highEndNewTime = highEndNewDist / hipHighEndOriginalSpeed;

            float hipHighNewTimeFraction = funcUtils::getFraction(0, startHighNewTime + highEndNewTime, startHighNewTime);
            float hipHighNewTime = funcUtils::lerp(baseEETargetCurve.getTRange()[0], baseEETargetCurve.getTRange()[1], hipHighNewTimeFraction);

            int hipHighPointIndex_hip = hipTrCurve.getClosestPointIndex(hipHighOriginalTime);
            hipTrCurve.displacePointT(hipHighPointIndex_hip, hipHighNewTime, true);
            hipRotAnglCurve.displacePointT(hipHighPointIndex_hip, hipHighNewTime, true);
            hipRotAxCurve.displacePointT(hipHighPointIndex_hip, hipHighNewTime, false);

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
            float originalTDiff = fallCurve.getTRange()[1] - fallCurve.getTRange()[0];
            float baseEEOriginalXYFallDist = glm::length(glm::vec2(baseEEOriginalCurve.getEnd() - baseEEOriginalCurve.evalCurve(hipHighOriginalTime)));
            float baseEETargetXYFallDist = glm::length(glm::vec2(baseEETargetCurve.getEnd() - baseEETargetCurve.evalCurve(hipHighNewTime)));
            float xyFallDistRatio = baseEETargetXYFallDist / baseEEOriginalXYFallDist;
            float calcTDiff = originalTDiff * xyFallDistRatio;

            // con el t encontrado y la aceleracion calculamos el punto final
            glm::vec3 finalTrans;
            // si el t cae antes de que termine la curva original
            float calcT = hipTrCurve.getTValue(hipHighPointIndex_hip) + calcTDiff;
            if (calcT <= hipTrCurve.getTRange()[1]) {
                finalTrans = hipTrCurve.evalCurve(calcT);
            }
            else {// si no
                finalTrans = hipTrCurve.getEnd();
                glm::vec3 calcVel = hipTrCurve.getVelocity(hipTrCurve.getTRange()[1]);
                int accSteps = 15;
                for (int i = 1; i <= accSteps; i++) {
                    finalTrans += calcVel * ((float)i / accSteps);
                    calcVel += fallAcc * calcTDiff * ((float)i / accSteps);
                }
            }
            std::vector<glm::vec3> newPoints;
            std::vector<float> newTimes;
            for (int i = 0; i < hipTrCurve.getNumberOfPoints(); i++) {
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
                hipHighNewTime, hipHighOriginalTime));
            hipTrFinalCurve.setCurvePoint(hipHighPointIndex_hip, hipHighAdjustedZ);

            m_tgData.pointIndexes.clear();
            m_tgData.pointIndexes.reserve(hipTrCurveAdjustedFall.getNumberOfPoints());
            m_tgData.baseVelocities.clear();
            m_tgData.baseVelocities.reserve(hipTrCurveAdjustedFall.getNumberOfPoints());
            m_tgData.minValues.clear();
            m_tgData.minValues.reserve(hipTrCurveAdjustedFall.getNumberOfPoints() * 3);
            for (int i = 1; i < hipTrCurveAdjustedFall.getNumberOfPoints(); i++) {
                // Los extremos de la curva se mantendran fijos.
                if (i != hipHighPointIndex_hip) {
                    m_tgData.pointIndexes.push_back(i);
                    for (int i = 0; i < 3; i++) {
                        m_tgData.minValues.push_back(std::numeric_limits<float>::min());
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
                m_tgData.baseVelocities.push_back(hipTrCurveAdjustedFall.getVelocity(hipTrCurveAdjustedFall.getTValue(pIndex)));
            }

            // seteo de otros parametros (TODO)
            m_tgData.descentRate;
            m_tgData.maxIterations = 100;
            m_tgData.varCurve = &hipTrFinalCurve;
            m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, initialArgs);

            float transitionTime = config->getCurrentReproductionTime();
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


    void TrajectoryGenerator::generateNewTrajectories(AnimationIndex animIndex,
        ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        // las trayectorias anteriores siempre deben llegar hasta el currentFrame

        IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
        float xyRotationAngle = glm::orientedAngle(glm::vec3(config->getHipTrajectoryData()->getOriginalFrontVector(), 0),
            glm::vec3(m_ikRig->getFrontVector(),0), glm::vec3(0, 0, 1));


        bool allStatic = true;
        for (int i = 0; i < m_ikChains.size(); i++) {
            ChainIndex ikChain = m_ikChains[i];
            JointIndex eeIndex = m_ikRig->getIKChain(ikChain)->getJoints().back();
            generateEETrajectory(ikChain, config, xyRotationAngle, transformManager, staticMeshManager);
        }

        // queda setear la trayectoria de la cadera
        generateHipTrajectory(config, xyRotationAngle, transformManager, staticMeshManager);  
    }



    
}