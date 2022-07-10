#include "TrajectoryGenerator.hpp"
#include "IKRig.hpp"
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
                dataPtr->baseCurve->getLeftHandVelocity(tVal));
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
            2 * (dataPtr->varCurve->getLeftHandVelocity(tVal)[coordIndex] - dataPtr->baseCurve->getLeftHandVelocity(tVal)[coordIndex])
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

    void setNewTrajectories(AnimationIndex animIndex, std::vector<ChainIndex> ikChains) {
        // las trayectorias anteriores siempre deben llegar hasta el currentFrame
        // se necesitan para cada ee su posicion actual y la curva base, ambos en espacio global
        // se usa "animationTime" que corresponde al tiempo de la aplicacion modificado con el playRate (-- distinto a samplingTime--)
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

    void TrajectoryGenerator::generateStaticTrajectory(ChainIndex ikChain, IKRigConfig* config,
        glm::vec3 globalEEPos,
        ComponentManager<TransformComponent>* transformManager,
        ComponentManager<StaticMeshComponent>* staticMeshManager) {
        EETrajectoryData* trData = config->getTrajectoryData(ikChain);
        FrameIndex nextFrame = config->getNextFrameIndex();
        FrameIndex currentFrame = config->getCurrentFrameIndex();
        float initialTime = config->getAnimationTime(config->getTimeStamps()[currentFrame]);
        glm::vec3 initialPos = trData->savedGlobalPositions[currentFrame];
        // chequear si hay una curva previa generada
        if (!trData->targetGlblTrajectory.inTRange(initialTime)) {
            initialPos = glm::vec3(glm::vec2(globalEEPos),
                m_environmentData.getTerrainHeight(glm::vec2(globalEEPos), transformManager, staticMeshManager));
        }
        float finalTime = config->getTimeStamps()[nextFrame];
        for (FrameIndex f = nextFrame + 1; f < config->getTimeStamps().size(); f++) {
            if (trData->supportFrames[f]) { finalTime = config->getTimeStamps()[f]; }
            else { break; }
        }
        finalTime = config->getAnimationTime(finalTime);
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

        // buscamos el frame en el que comienza la trayectoria
        FrameIndex initialFrame = currentFrame;
        for (int i = 0; i < config->getTimeStamps().size(); i++) {
            initialFrame = 0 < initialFrame ? initialFrame - 1 : config->getTimeStamps().size() - 1;
            if (trData->supportFrames[initialFrame]) { break; }
        }
        MONA_ASSERT(initialFrame != currentFrame, "TrajectoryGenerator: Trajectory must have a starting point.");
        int repOffsetStart = initialFrame < currentFrame ? 0 : -1;
        float initialTime = config->getAnimationTime(config->getTimeStamps()[initialFrame], repOffsetStart);
        // buscamos el frame en el que termina la trayectoria
        FrameIndex finalFrame = currentFrame;
        FrameIndex testFrame = currentFrame;
        for (int i = 0; i < config->getTimeStamps().size(); i++) {
            testFrame = testFrame < config->getTimeStamps().size() - 1 ? testFrame + 1 : 0;
            if (trData->supportFrames[testFrame]) { break; }
            finalFrame = testFrame;
        }
        int repOffsetEnd = currentFrame <= finalFrame ? 0 : 1;
        float finalTime = config->getAnimationTime(config->getTimeStamps()[finalFrame], repOffsetEnd);
        // ahora se extrae una subcurva  de la trayectoria original para generar la trayectoria requerida
        // hay que revisar si la trayectoria viene en una sola pieza
        LIC<3> sampledCurve;
        if (initialFrame < finalFrame) {
            sampledCurve = trData->originalGlblTrajectory.sample(config->getTimeStamps()[initialFrame], config->getTimeStamps()[finalFrame]);
        }
        else {
            LIC<3> part1 = trData->originalGlblTrajectory.sample(config->getTimeStamps()[initialFrame], config->getTimeStamps().back());
            LIC<3> part2 = trData->originalGlblTrajectory.sample(config->getTimeStamps()[0], config->getTimeStamps()[finalFrame]);
            part2.offsetTValues(config->getAnimationDuration());
            part2.translate(-(part2.getStart() - part1.getEnd()));
            sampledCurve = LIC<3>::join(part1, part2);
        }
        float tOffset = -config->getTimeStamps()[initialFrame] + initialTime;
        sampledCurve.offsetTValues(tOffset);
        glm::vec3 sampledCurveStart = sampledCurve.getCurvePoint(0);
        glm::vec3 sampledCurveEnd = sampledCurve.getCurvePoint(sampledCurve.getNumberOfPoints() - 1);
        glm::vec3 initialPos = trData->savedGlobalPositions[initialFrame];
        // chequear si hay una curva previa generada
        if (!trData->targetGlblTrajectory.inTRange(initialTime)) {
            float referenceTime = config->getAnimationTime(config->getTimeStamps()[currentFrame]);
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

        sampledCurveStart = sampledCurve.getCurvePoint(0);
        sampledCurveEnd = sampledCurve.getCurvePoint(sampledCurve.getNumberOfPoints() - 1);
        // escalarla y moverla para que calce con las posiciones
        float origLength = glm::distance(sampledCurveStart, sampledCurveEnd); // actualmente parte en el origen
        float targetLength = glm::distance(initialPos, finalPos);
        sampledCurve.scale(glm::vec3(targetLength/origLength));
        sampledCurve.translate(initialPos);

        trData->trajectoryType = TrajectoryType::DYNAMIC;
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




    
}