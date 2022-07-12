#include "TrajectoryGenerator.hpp"
#include "../Core/GlmUtils.hpp"
#include "glm/gtx/rotate_vector.hpp"
#include "glm/gtx/vector_angle.hpp"

namespace Mona{

    // funciones para descenso de gradiente
    template <int D>
    std::function<float(const std::vector<float>&, TGData<D>*)> term1Function =
        [](const std::vector<float>& varPCoord, TGData<D>* dataPtr)->float {
        float result = 0;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            int pIndex = dataPtr->pointIndexes[i];
            float tVal = dataPtr->varCurve->getTValue(pIndex);
            result += glm::distance2(dataPtr->varCurve->getLeftHandVelocity(tVal),
                dataPtr->baseVelocities[i]);
        }
        return result;
    };

    template <int D>
    std::function<float(const std::vector<float>&, int, TGData<D>*)> term1PartialDerivativeFunction =
        [](const std::vector<float>& varPCoord, int varIndex, TGData<D>* dataPtr)->float {
        int pIndex = dataPtr->pointIndexes[varIndex / D];
        int coordIndex = varIndex % D;
        float tVal = dataPtr->varCurve->getTValue(pIndex);
        float result = pIndex != 0 ?
            2 * (dataPtr->varCurve->getLeftHandVelocity(tVal)[coordIndex] - dataPtr->baseVelocities[varIndex / D][coordIndex])
            / (dataPtr->varCurve->getTValue(pIndex) - dataPtr->varCurve->getTValue(pIndex - 1)) : 0;

        return result;
    };

    template <int D>
    std::function<void(std::vector<float>&, TGData<D>*)>  postDescentStepCustomBehaviour = 
        [](std::vector<float>& varPCoord, TGData<D>* dataPtr)->void {
        glm::vec<D, float> newPos;
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
       
        // descenso para angulos (dim 1)
        FunctionTerm<TGData<1>> dim1Term(term1Function<1>, term1PartialDerivativeFunction<1>);
        m_gradientDescent_dim1 = GradientDescent<TGData<1>>({ dim1Term }, 0, &m_tgData_dim1, postDescentStepCustomBehaviour<1>);

        // descenso para posiciones (dim 3)
        FunctionTerm<TGData<3>> dim3Term(term1Function<3>, term1PartialDerivativeFunction<3>);
        m_gradientDescent_dim3 = GradientDescent<TGData<3>>({ dim3Term }, 0, &m_tgData_dim3, postDescentStepCustomBehaviour<3>);
    }

    glm::vec3 removeHipMotion(glm::vec3 pseudoModelSpacePos, float animationLoopTime, IKRigConfig* config) {
        auto hipTrData = config->getHipTrajectoryData();
        float pseudoMSHipRotAngle = hipTrData->originalRotationAngles.evalCurve(animationLoopTime)[0];
        glm::vec3 pseudoMSHipRotAxis = hipTrData->originalRotationAxes.evalCurve(animationLoopTime);
        glm::fquat pseudoMSHipRot = glm::fquat(pseudoMSHipRotAngle, pseudoMSHipRotAxis);
        glm::vec3 pseudoMSHipTr = hipTrData->originalGlblTranslations.evalCurve(animationLoopTime);
        glm::mat4 hipMotion = glmUtils::translationToMat4(pseudoMSHipTr)*
            glmUtils::rotationToMat4(pseudoMSHipRot);
        return glm::inverse(hipMotion) * glm::vec4(pseudoModelSpacePos, 1);

    }

    glm::vec3 TrajectoryGenerator::calcStrideStartingPoint(glm::vec3 referencePoint, float targetDistance, 
        glm::vec2 targetDirection, int stepNum,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        glm::vec2 refPointBase(referencePoint[0], referencePoint[1]);
        std::vector<glm::vec3> collectedPoints;
        collectedPoints.reserve(stepNum);
        for (int i = 1; i <= stepNum; i++) {
            glm::vec2 testPoint = refPointBase + targetDirection * targetDistance * ((float)i / stepNum);
            collectedPoints.push_back(glm::vec3(testPoint, m_environmentData.getTerrainHeight(testPoint,
                    transformManager, staticMeshManager)));
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

    std::vector<glm::vec3> TrajectoryGenerator::calcStrideData(glm::vec3 startingPoint, float targetDistance, 
        glm::vec2 targetDirection, int stepNum,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        std::vector<glm::vec3> collectedPoints;
        collectedPoints.reserve(stepNum);
        for (int i = 1; i <= stepNum; i++) {
            glm::vec2 testPoint = glm::vec2(startingPoint) + targetDirection * targetDistance * ((float)i / stepNum);
            collectedPoints.push_back(glm::vec3(testPoint, 
                m_environmentData.getTerrainHeight(testPoint, transformManager, staticMeshManager)));
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

    std::pair<FrameIndex, FrameIndex> TrajectoryGenerator::calcTrajectoryFrameRange(IKRigConfig* config, ChainIndex ikChain, 
        TrajectoryType trajectoryType) {
        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex nextFrame = config->getNextFrameIndex();
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        if (trajectoryType==TrajectoryType::STATIC) { // estatica
            FrameIndex finalFrame = nextFrame;
            for (int i = 0; i < config->getFrameNum()-1; i++) {
                if (!trData->supportFrames[config->getFrameNum() % (finalFrame + 1)]) { break; }
                finalFrame = config->getFrameNum() % (finalFrame + 1);
            }
            return { currentFrame, finalFrame };
        }
        else { // dinamica
            // buscamos el frame en el que comienza la trayectoria
            FrameIndex initialFrame = currentFrame;
            for (int i = 0; i < config->getFrameNum() -1; i++) {
                if (trData->supportFrames[initialFrame]) { break; }
                initialFrame = 0 < initialFrame ? initialFrame - 1 : config->getFrameNum() - 1;
            }
            MONA_ASSERT(initialFrame != nextFrame, "TrajectoryGenerator: Trajectory must have a starting point.");

            // buscamos el frame en el que termina la trayectoria
            FrameIndex finalFrame = nextFrame;
            for (int i = 0; i < config->getFrameNum() - 1; i++) {
                if (trData->supportFrames[config->getFrameNum() %(finalFrame+1)]) { break; }
                finalFrame = config->getFrameNum() % (finalFrame + 1);
            }
            return { initialFrame, finalFrame };
        }
    }

    void TrajectoryGenerator::generateStaticTrajectory(ChainIndex ikChain, IKRigConfig* config,
        glm::vec3 globalEEPos,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex nextFrame = config->getNextFrameIndex();
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        std::pair<FrameIndex, FrameIndex> frameRange = calcTrajectoryFrameRange(config, ikChain, TrajectoryType::STATIC);
        FrameIndex initialFrame = frameRange.first;
        FrameIndex finalFrame = frameRange.second;
        float initialTime = config->getReproductionTime(initialFrame);
        int repOffsetEnd = initialFrame <= finalFrame ? 0 : 1;
        float finalTime = config->getReproductionTime(finalFrame, repOffsetEnd);
        glm::vec3 initialPos = trData->savedGlobalPositions[initialTime];
        // chequear si hay una curva previa generada
        if (!trData->targetGlblTrajectory.inTRange(initialTime)) {
            initialPos = glm::vec3(glm::vec2(globalEEPos),
                m_environmentData.getTerrainHeight(glm::vec2(globalEEPos), transformManager, staticMeshManager));
        }
        trData->targetGlblTrajectory = LIC<3>({ initialPos, initialPos }, { initialTime, finalTime });
        trData->trajectoryType = TrajectoryType::STATIC;
    }

    void TrajectoryGenerator::generateDynamicTrajectory(ChainIndex ikChain, IKRigConfig* config,
        glm::vec3 globalEEPos, float rotationAngle,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {

        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex nextFrame = config->getNextFrameIndex();
        FrameIndex currentFrame = config->getCurrentFrameIndex();

        std::pair<FrameIndex, FrameIndex> frameRange = calcTrajectoryFrameRange(config, ikChain, TrajectoryType::DYNAMIC);
        FrameIndex initialFrame = frameRange.first;
        FrameIndex finalFrame = frameRange.second;

        int repOffsetStart = initialFrame < currentFrame ? 0 : -1;
        float initialTime = config->getReproductionTime(initialFrame, repOffsetStart);

        int repOffsetEnd = currentFrame < finalFrame ? 0 : 1;
        float finalTime = config->getReproductionTime(finalFrame, repOffsetEnd);

        // ahora se extrae una subcurva  de la trayectoria original para generar la trayectoria requerida
        // hay que revisar si la trayectoria viene en una sola pieza
        LIC<3> sampledCurve;
        if (initialFrame < finalFrame) {
            sampledCurve = trData->originalGlblTrajectory.sample(config->getAnimationTime(initialFrame), config->getAnimationTime(finalFrame));
        }
        else {
            LIC<3> part1 = trData->originalGlblTrajectory.sample(config->getAnimationTime(initialFrame), config->getAnimationTime(config->getFrameNum()-1));
            LIC<3> part2 = trData->originalGlblTrajectory.sample(config->getAnimationTime(0), config->getAnimationTime(finalFrame));
            part2.offsetTValues(config->getAnimationDuration());
            part2.translate(-(part2.getStart() - part1.getEnd()));
            sampledCurve = LIC<3>::join(part1, part2);
        }
        float tOffset = -config->getAnimationTime(initialFrame) + initialTime;
        sampledCurve.offsetTValues(tOffset);
        glm::vec3 sampledCurveStart = sampledCurve.getCurvePoint(0);
        glm::vec3 sampledCurveEnd = sampledCurve.getCurvePoint(sampledCurve.getNumberOfPoints() - 1);
        glm::vec3 initialPos = trData->savedGlobalPositions[initialFrame];
        // chequear si hay una curva previa generada
        if (!trData->targetGlblTrajectory.inTRange(initialTime)) {
            float referenceTime = config->getReproductionTime(currentFrame);
            glm::vec3 sampledCurveReferencePoint = sampledCurve.evalCurve(referenceTime);
            float floorDistanceToStart = glm::distance(glm::vec2(sampledCurveStart), glm::vec2(sampledCurveReferencePoint));
            glm::vec3 floorReferencePoint(glm::vec2(globalEEPos), m_environmentData.getTerrainHeight(glm::vec2(globalEEPos),
                transformManager, staticMeshManager));
            glm::vec2 sampledCurveReferenceDirection = glm::vec2(sampledCurveReferencePoint) - glm::vec2(sampledCurveStart);
            glm::vec2 targetDirection = -glm::normalize(glm::rotate(sampledCurveReferenceDirection, rotationAngle));
            initialPos = calcStrideStartingPoint(floorReferencePoint, floorDistanceToStart,
                targetDirection, 4, transformManager, staticMeshManager);
        }
        float targetDistance = glm::distance(sampledCurveStart, sampledCurveEnd);
        glm::vec3 originalDirection = glm::normalize(sampledCurveEnd - sampledCurveStart);
        glm::vec2 targetXYDirection = glm::rotate(glm::vec2(originalDirection), rotationAngle);
        std::vector<glm::vec3> strideData = calcStrideData(initialPos, targetDistance, targetXYDirection, 8, transformManager, staticMeshManager);
        if (strideData.size() == 0) { // si no es posible avanzar por la elevacion del terreno
            generateStaticTrajectory(ikChain, config, globalEEPos, transformManager, staticMeshManager);
            return;
        }
        glm::vec3 finalPos = strideData.back();
        
        // mover la curva al origen
        sampledCurve.translate(-sampledCurveStart);
        // rotarla la subtrayectoria para que quede en linea con las pos inicial y final
        glm::fquat targetRotation = glm::identity<glm::fquat>();
        glm::vec3 targetDirection = glm::normalize(finalPos - initialPos);
        if (originalDirection != targetDirection) {
            glm::vec3 rotAxis = glm::normalize(glm::cross(originalDirection, targetDirection));
            float rotAngle = glm::orientedAngle(originalDirection, targetDirection, rotAxis);
            targetRotation = glm::fquat(rotAngle, rotAxis);
        }
        sampledCurve.rotate(targetRotation);        

        // escalarla y moverla para que calce con las posiciones
        float origLength = glm::distance(sampledCurveStart, sampledCurveEnd); // actualmente parte en el origen
        float targetLength = glm::distance(initialPos, finalPos);
        sampledCurve.scale(glm::vec3(targetLength/origLength));
        sampledCurve.translate(initialPos);

        // setear los minimos de altura y aplicar el descenso de gradiente
        m_tgData_dim3.pointIndexes.clear();
        m_tgData_dim3.pointIndexes.reserve(sampledCurve.getNumberOfPoints());
        m_tgData_dim3.baseVelocities.clear();
        m_tgData_dim3.baseVelocities.reserve(sampledCurve.getNumberOfPoints());
        m_tgData_dim3.minValues.clear();
        m_tgData_dim3.minValues.reserve(sampledCurve.getNumberOfPoints() * 3);
        int stridePointInd = 0;
        float stridePointPreviousDist = glm::distance2(glm::vec2(sampledCurve.getCurvePoint(0)), glm::vec2(strideData[stridePointInd]));
        std::vector<std::pair<int, int>> curvePointIndex_stridePointIndex;
        for (int i = 1; i < sampledCurve.getNumberOfPoints(); i++) {
            // Los extremos de la curva se mantendran fijos.
            m_tgData_dim3.pointIndexes.push_back(i);
            float currDist = glm::distance2(glm::vec2(sampledCurve.getCurvePoint(i)), glm::vec2(strideData[stridePointInd]));
            if (stridePointPreviousDist < currDist) {
                // si la distancia del punto actual es mayor, entonces el anterior se escoge como el mas cercano
                curvePointIndex_stridePointIndex.push_back({i-1, stridePointInd});
                stridePointInd += 1;
                if (stridePointInd == strideData.size()) { break; }
            }
            else if (i == sampledCurve.getNumberOfPoints() - 1) {
                curvePointIndex_stridePointIndex.push_back({ i, stridePointInd });
            }
        }
        for (int i = 0; i < m_tgData_dim3.pointIndexes.size()*3;i++) {
            m_tgData_dim3.minValues.push_back(std::numeric_limits<float>::min());
        }
        for (int i = 0; i < curvePointIndex_stridePointIndex.size(); i++){
            int curvePointIndex = curvePointIndex_stridePointIndex[i].first;
            int stridePointIndex = curvePointIndex_stridePointIndex[i].second;
            // asignamos el valor minimo de z permitido al punto designado y a sus vecinos inmediatos
            if (0 < curvePointIndex) {
                m_tgData_dim3.minValues[(curvePointIndex - 1) * 3 + 2] = strideData[stridePointIndex][2];
            }
            m_tgData_dim3.minValues[(curvePointIndex) * 3 + 2] = strideData[stridePointIndex][2];
            if (curvePointIndex < (m_tgData_dim3.pointIndexes.size() - 1)) {
                m_tgData_dim3.minValues[(curvePointIndex + 1) * 3 + 2] = strideData[stridePointIndex][2];
            }
        }
        // parametros iniciales y velocidades base
        std::vector<float> initialArgs(m_tgData_dim3.pointIndexes.size() * 3);
        for (int i = 0; i < m_tgData_dim3.pointIndexes.size(); i++) {
            int pIndex = m_tgData_dim3.pointIndexes[i];
            for (int j = 0; j < 2; j++) {
                initialArgs[i * 3 + j] = sampledCurve.getCurvePoint(pIndex)[j];
            }
            m_tgData_dim3.baseVelocities.push_back(sampledCurve.getLeftHandVelocity(sampledCurve.getTValue(pIndex)));
        }

        trData->targetGlblTrajectory = sampledCurve;
        trData->trajectoryType = TrajectoryType::DYNAMIC;

        // seteo de otros parametros (TODO)
        m_tgData_dim3.descentRate;
        m_tgData_dim3.maxIterations = 100;

        m_tgData_dim3.varCurve = &trData->targetGlblTrajectory;
        m_gradientDescent_dim3.computeArgsMin(m_tgData_dim3.descentRate, m_tgData_dim3.maxIterations, initialArgs);

        
    }

    void TrajectoryGenerator::generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, 
        glm::vec3 globalEEPos,
        float rotationAngle,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex nextFrame = config->getNextFrameIndex();
        // chequemos que tipo de trayectoria hay que crear (estatica o dinamica)
        // si es estatica
        if (trData->supportFrames[nextFrame]) {
            generateStaticTrajectory(ikChain, config, globalEEPos, transformManager, staticMeshManager);
        } // si es dinamica
        else {
            generateDynamicTrajectory(ikChain, config, globalEEPos, rotationAngle, transformManager, staticMeshManager);
        }        
    }

    float TrajectoryGenerator::calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float reproductionTime, int stepNum) {
        HipTrajectoryData* hipTrData = config->getHipTrajectoryData();
        float animationTime = config->getAnimationTime(reproductionTime);

        // distancias originales de ee's con cadera
        std::vector<float> origDistances(m_ikChains.size());
        std::vector<float> origZDistances(m_ikChains.size());
        float maxValue_zDist = std::numeric_limits<float>::min();
        int maxValIndex_zDist = -1;
        for (int i = 0; i < m_ikChains.size(); i++) {
            EETrajectoryData* trData = config->getTrajectoryData(m_ikChains[i]);
            glm::vec3 hipPoint = hipTrData->originalGlblTrajectory.evalCurve(animationTime);
            glm::vec3 eePoint = trData->originalGlblTrajectory.evalCurve(animationTime);
            origDistances[i] = glm::distance(hipPoint, eePoint);
            origZDistances[i] = abs(hipPoint[2] - eePoint[2]);
            if (maxValue_zDist < origZDistances[i]) {
                maxValIndex_zDist = origZDistances[i];
                maxValIndex_zDist = i;
            }
        }
        // buscamos el maximo valor para z, tal que no se sobrepase ninguna de las distancias originales
        EETrajectoryData* trData = config->getTrajectoryData(m_ikChains[maxValIndex_zDist]);
        float adjustedHeight = trData->targetGlblTrajectory.evalCurve(reproductionTime)[2] + maxValue_zDist;
        for (int i = 0; i < stepNum; i++) {
            float maxDistDiff = std::numeric_limits<float>::min();
            for (int j = 0; j < m_ikChains.size(); j++) {
                trData = config->getTrajectoryData(m_ikChains[j]);
                float distDiff = 0;
                float currentDistance = 0;
                if (trData->targetGlblTrajectory.inTRange(reproductionTime)) {
                    currentDistance = glm::distance(trData->targetGlblTrajectory.evalCurve(reproductionTime),
                        glm::vec3(basePoint, adjustedHeight));
                }
                else {
                    float closestTime = std::min(abs(trData->targetGlblTrajectory.getTRange()[0] - reproductionTime),
                        abs(trData->targetGlblTrajectory.getTRange()[1] - reproductionTime));
                    currentDistance = abs(trData->targetGlblTrajectory.evalCurve(closestTime)[2] - adjustedHeight);
                }
                distDiff = currentDistance - origDistances[j];
                
                if (distDiff > maxDistDiff) {
                    maxDistDiff = distDiff;
                }
            }
            // si nos pasamos en distancia
            if (maxDistDiff > 0) {
                adjustedHeight -= maxDistDiff * 0.7;
            }
            else {
                adjustedHeight += abs(maxDistDiff) * 0.7;
            }
        }
        return adjustedHeight;
    }

    void TrajectoryGenerator::generateHipTrajectory(IKRigConfig* config, float rotationAngle, glm::vec3 globalHipPos,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {

        bool allStatic = true;
        for (int i = 0; i < m_ikChains.size(); i++) {
            ChainIndex ikChain = m_ikChains[i];
            if (config->getTrajectoryData(ikChain)->trajectoryType == TrajectoryType::DYNAMIC) { allStatic = false; }
        }

        HipTrajectoryData* hipTrData = config->getHipTrajectoryData();
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        
        if (allStatic) {
            float initialTime = config->getReproductionTime(currentFrame);
            float initialTime_anim = config->getAnimationTime(currentFrame);
            glm::vec2 basePoint(globalHipPos);
            glm::vec3 initialPos = hipTrData->savedGlobalPositions[initialTime];
            // chequear si hay una curva previa generada
            if (!hipTrData->targetGlblTranslations.inTRange(initialTime)) {
                // distancias originales de ee's con cadera
                std::vector<float> origDistances(m_ikChains.size());
                std::vector<float> origZDiffs(m_ikChains.size());
                float maxValue_zDist = std::numeric_limits<float>::min();
                int maxValIndex_zDist = -1;
                for (int i = 0; i < m_ikChains.size(); i++) {
                    EETrajectoryData* trData = config->getTrajectoryData(m_ikChains[i]);
                    glm::vec3 hipPoint = hipTrData->originalGlblTrajectory.evalCurve(initialTime_anim);
                    glm::vec3 eePoint = trData->originalGlblTrajectory.evalCurve(initialTime_anim);
                    origDistances[i] = glm::distance(hipPoint, eePoint);
                    origZDiffs[i] = hipPoint[2] - eePoint[2];
                    if (maxValue_zDist < abs(origZDiffs[i])) {
                        maxValIndex_zDist = abs(origZDiffs[i]);
                        maxValIndex_zDist = i;
                    }
                }
                // buscamos el maximo valor para z, tal que no se sobrepase ninguna de las distancias originales
                EETrajectoryData* trData = config->getTrajectoryData(m_ikChains[maxValIndex_zDist]);
                float adjustmentValue = maxValue_zDist/2;
                float adjustedHeight = trData->targetGlblTrajectory.evalCurve(initialTime)[2];
                for (int i = 0; i < 10;i ++) {
                    bool increased = false;
                    for (int j = 0; j < m_ikChains.size(); j++) {
                        trData = config->getTrajectoryData(m_ikChains[j]);
                        float currentDistance = glm::distance(trData->targetGlblTrajectory.evalCurve(initialTime),
                            glm::vec3(basePoint, adjustedHeight));
                        if (currentDistance < origDistances[j]) {
                            adjustedHeight += adjustmentValue;
                            adjustmentValue /= 2;
                            increased = true;
                            break;
                        }
                    }
                    if (!increased) {
                        adjustedHeight -= adjustmentValue;
                        adjustmentValue /= 2;
                    }
                }
            }
        }
        else {
            for (int i = 0; i < m_ikChains.size(); i++) {
            }
        }
    }


    void TrajectoryGenerator::generateNewTrajectories(AnimationIndex animIndex,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        // las trayectorias anteriores siempre deben llegar hasta el currentFrame

        IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);

        glm::mat4 baseTransform = transformManager->GetComponentPointer(m_ikRig->getTransformHandle())->GetModelMatrix();
        std::vector<glm::vec3> globalPositions = config->getCustomSpacePositions(baseTransform, true);
        float xyRotationAngle = glm::orientedAngle(glm::vec3(config->getHipTrajectoryData()->originalFrontVector, 0),
            glm::vec3(m_ikRig->getFrontVector(),0), glm::vec3(0, 0, 1));


        bool allStatic = true;
        for (int i = 0; i < m_ikChains.size(); i++) {
            ChainIndex ikChain = m_ikChains[i];
            JointIndex eeIndex = m_ikRig->getIKChain(ikChain)->getJoints().back();
            generateEETrajectory(ikChain, config, globalPositions[eeIndex], xyRotationAngle, transformManager, staticMeshManager);
        }

        // queda setear la trayectoria de la cadera
        generateHipTrajectory(config, globalPositions[m_ikRig->getHipJoint()], xyRotationAngle, transformManager, staticMeshManager);  
    }



    
}