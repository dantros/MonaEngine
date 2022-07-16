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

    glm::vec3 TrajectoryGenerator::calcStrideStartingPoint(float supportHeight,
        glm::vec3 referencePoint, float targetDistance, 
        glm::vec2 targetDirection, int stepNum,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
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
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
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

    void TrajectoryGenerator::generateStaticTrajectory(LIC<3> baseCurve,
        ChainIndex ikChain, IKRigConfig* config,
        glm::vec3 globalEEPos,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        EETrajectoryData* trData = config->getTrajectoryData(ikChain);

        FrameIndex initialFrame = config->getCurrentFrameIndex();
        float initialTime = config->getReproductionTime(initialFrame);
        glm::vec3 initialPos = trData->getSavedGlobalPosition(initialFrame);
        // chequear si hay una curva previa generada
        if (!trData->getTargetGlblTrajectory().getEETrajectory().inTRange(initialTime)) {
            float calcHeight = m_environmentData.getTerrainHeight(glm::vec2(globalEEPos), transformManager, staticMeshManager);
            initialPos = glm::vec3(glm::vec2(globalEEPos), trData->getSupportHeight(initialFrame) + calcHeight);
        }
        // correccion de posicion y llevar a reproduction time
        baseCurve.translate(-baseCurve.getStart());
        baseCurve.translate(initialPos);
        baseCurve.offsetTValues(-baseCurve.getTValue(0));
        baseCurve.offsetTValues(initialTime);
        trData->setTargetGlblTrajectory(baseCurve, TrajectoryType::STATIC);
    }

    void TrajectoryGenerator::generateDynamicTrajectory(LIC<3> baseCurve, 
        ChainIndex ikChain, IKRigConfig* config,
        glm::vec3 globalEEPos, float rotationAngle,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {

        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        FrameIndex initialFrame = config->getFrame(baseCurve.getTValue(0));

        // llevar a reproduction time
        float currentAnimTime = config->getAnimationTime(currentFrame);
        float currentRepTime = config->getReproductionTime(currentFrame);
        baseCurve.offsetTValues(-currentAnimTime);
        baseCurve.offsetTValues(currentRepTime);      

        float initialRepTime = baseCurve.getTValue(0);
        glm::vec3 initialPos = trData->getSavedGlobalPosition(initialFrame);
        // chequear si hay una curva previa generada
        if (!trData->getTargetGlblTrajectory().getEETrajectory().inTRange(initialRepTime)) {
            float referenceTime = config->getReproductionTime(currentFrame);
            glm::vec3 sampledCurveReferencePoint = baseCurve.evalCurve(referenceTime);
            float floorDistanceToStart = glm::distance(glm::vec2(baseCurve.getStart()), glm::vec2(sampledCurveReferencePoint));
            glm::vec3 floorReferencePoint(glm::vec2(globalEEPos), m_environmentData.getTerrainHeight(glm::vec2(globalEEPos),
                transformManager, staticMeshManager));
            glm::vec2 sampledCurveReferenceDirection = glm::vec2(sampledCurveReferencePoint) - glm::vec2(baseCurve.getStart());
            glm::vec2 targetDirection = -glm::normalize(glm::rotate(sampledCurveReferenceDirection, rotationAngle));
            float supportHeight = trData->getSupportHeight(initialFrame);
            initialPos = calcStrideStartingPoint(supportHeight, floorReferencePoint, floorDistanceToStart,
                targetDirection, 4, transformManager, staticMeshManager);
        }
        float targetDistance = glm::distance(baseCurve.getStart(), baseCurve.getEnd())*m_ikRig->getStrideFactor();
        glm::vec3 originalDirection = glm::normalize(baseCurve.getEnd() - baseCurve.getStart());
        glm::vec2 targetXYDirection = glm::rotate(glm::vec2(originalDirection), rotationAngle);
        float supportHeight = trData->getSupportHeight(initialFrame);
        std::vector<glm::vec3> strideData = calcStrideData(supportHeight, initialPos, targetDistance, targetXYDirection, 8, transformManager, staticMeshManager);
        if (strideData.size() == 0) { // si no es posible avanzar por la elevacion del terreno
            LIC<3> staticBaseCurve({ globalEEPos, globalEEPos }, { currentAnimTime, currentAnimTime + config->getAnimationDuration() });
            generateStaticTrajectory(staticBaseCurve, ikChain, config, globalEEPos, transformManager, staticMeshManager);
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
        m_tgData_dim3.pointIndexes.clear();
        m_tgData_dim3.pointIndexes.reserve(baseCurve.getNumberOfPoints());
        m_tgData_dim3.baseVelocities.clear();
        m_tgData_dim3.baseVelocities.reserve(baseCurve.getNumberOfPoints());
        m_tgData_dim3.minValues.clear();
        m_tgData_dim3.minValues.reserve(baseCurve.getNumberOfPoints() * 3);
        int stridePointInd = 0;
        float stridePointPreviousDist = glm::distance2(glm::vec2(baseCurve.getCurvePoint(0)), glm::vec2(strideData[stridePointInd]));
        std::vector<std::pair<int, int>> curvePointIndex_stridePointIndex;
        for (int i = 1; i < baseCurve.getNumberOfPoints(); i++) {
            // Los extremos de la curva se mantendran fijos.
            m_tgData_dim3.pointIndexes.push_back(i);
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
        // valores iniciales y velocidades base
        std::vector<float> initialArgs(m_tgData_dim3.pointIndexes.size() * 3);
        for (int i = 0; i < m_tgData_dim3.pointIndexes.size(); i++) {
            int pIndex = m_tgData_dim3.pointIndexes[i];
            for (int j = 0; j < 2; j++) {
                initialArgs[i * 3 + j] = baseCurve.getCurvePoint(pIndex)[j];
            }
            m_tgData_dim3.baseVelocities.push_back(baseCurve.getLeftHandVelocity(baseCurve.getTValue(pIndex)));
        }

        // seteo de otros parametros (TODO)
        m_tgData_dim3.descentRate;
        m_tgData_dim3.maxIterations = 100;
        m_tgData_dim3.varCurve = &baseCurve;
        m_gradientDescent_dim3.computeArgsMin(m_tgData_dim3.descentRate, m_tgData_dim3.maxIterations, initialArgs);

        float transitionTime = config->getCurrentReproductionTime();
        if (trData->getTargetGlblTrajectory().getEETrajectory().inTRange(transitionTime)) {
            trData->setTargetGlblTrajectory(LIC<3>::transition(trData->getTargetGlblTrajectory().getEETrajectory(), 
                baseCurve, transitionTime), TrajectoryType::DYNAMIC);
        }
        else {
            trData->setTargetGlblTrajectory(baseCurve, TrajectoryType::DYNAMIC);
        }   
    }

    void TrajectoryGenerator::generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, 
        glm::vec3 globalEEPos,
        float rotationAngle,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex initialFrame = config->getCurrentFrameIndex();
        float initialAnimTime = config->getAnimationTime(initialFrame);
        float initialRepTime = config->getReproductionTime(initialFrame);
        FrameIndex nextFrame = config->getNextFrameIndex();
        if (config->getAnimationType() == AnimationType::IDLE) {
            LIC<3> baseCurve({ globalEEPos, globalEEPos }, { initialRepTime, initialRepTime + config->getAnimationDuration() });
            generateStaticTrajectory(baseCurve, ikChain, config, globalEEPos, transformManager, staticMeshManager);
            return;
        }
        EETrajectory originalTrajectory = trData->getSubTrajectory(initialAnimTime);
        LIC<3> baseCurve = originalTrajectory.getEETrajectory();
        if (originalTrajectory.isDynamic()) {
            generateDynamicTrajectory(baseCurve, ikChain, config, globalEEPos, rotationAngle, transformManager, staticMeshManager);
        }
        else {
            generateStaticTrajectory(baseCurve, ikChain, config, globalEEPos, transformManager, staticMeshManager);
        }    
    }

    float TrajectoryGenerator::calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float reproductionTime) {
        HipTrajectoryData* hipTrData = config->getHipTrajectoryData();
        float animationTime = config->getAnimationTime(reproductionTime);

        // distancias originales de ee's con cadera
        std::vector<float> origZDistances(m_ikChains.size());
        for (int i = 0; i < m_ikChains.size(); i++) {
            EETrajectoryData* trData = config->getTrajectoryData(m_ikChains[i]);
            glm::vec3 hipPoint = hipTrData->originalGlblTrajectory.evalCurve(animationTime);
            glm::vec3 eePoint = trData->originalGlblTrajectory.evalCurve(animationTime);
            origZDistances[i] = abs(hipPoint[2] - eePoint[2]);
        }

        // valor mas bajo de z para las tr actuales de los ee
        float minZ = std::numeric_limits<float>::max();
        int minZIndex = -1;
        EETrajectoryData* trData;
        for (int i = 0; i < m_ikChains.size(); i++) {
            trData = config->getTrajectoryData(m_ikChains[i]);
            float time = reproductionTime;
            if (!trData->targetGlblTrajectory.inTRange(time)) {
                float delta0 = abs(trData->targetGlblTrajectory.getTRange()[0] - reproductionTime);
                float delta1 = abs(trData->targetGlblTrajectory.getTRange()[1] - reproductionTime);
                time = trData->targetGlblTrajectory.getTRange()[0];
                if (delta1 < delta0) { time = trData->targetGlblTrajectory.getTRange()[1]; }
            }
            glm::vec3 eePoint = trData->targetGlblTrajectory.evalCurve(time);
            if (eePoint[2] < minZ) { 
                minZIndex = i;
                minZ = eePoint[2]; 
            }
        }
        float adjustedHeight = minZ + origZDistances[minZIndex];
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
        FrameIndex nextFrame = config->getNextFrameIndex();
        
        if (allStatic) {
            float initialTime = config->getReproductionTime(currentFrame);
            glm::vec2 basePoint(globalHipPos);
            glm::vec3 initialPos = hipTrData->savedGlobalPositions[initialTime];
            // chequear si hay una curva previa generada
            if (!hipTrData->targetGlblTranslations.inTRange(initialTime)) {
                initialPos = glm::vec3(basePoint, calcHipAdjustedHeight(config, basePoint, initialTime));
            }

        }
        else {
            HipTrajectoryData* hipTrData = config->getHipTrajectoryData();
            // buscamos la curva dinamica a la que le quede mas tiempo
            float tInfLimit = std::numeric_limits<float>::min();
            float tSupLimit = std::numeric_limits<float>::min();
            LIC<3>* baseDynTr;
            for (int i = 0; i < m_ikChains.size(); i++) {
                EETrajectoryData* trData = config->getTrajectoryData(m_ikChains[i]);
                float tCurrentSupLimit = trData->targetGlblTrajectory.getTRange()[1];
                if (trData->trajectoryType == TrajectoryType::DYNAMIC && tSupLimit < tCurrentSupLimit) {
                    tSupLimit = tCurrentSupLimit;
                    tInfLimit = trData->targetGlblTrajectory.getTRange()[0];
                    baseDynTr = &(trData->targetGlblTrajectory);
                }
            }
            LIC<3> sampleCurveTr = hipTrData->originalGlblTranslations.sample(tInfLimit, tSupLimit);
            LIC<1> sampleCurveRotAngl = hipTrData->originalRotationAngles.sample(tInfLimit, tSupLimit);
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