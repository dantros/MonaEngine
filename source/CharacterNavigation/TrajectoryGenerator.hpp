#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"
#include "ParametricCurves.hpp"
#include "IKRig.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;
    typedef int FrameIndex;

    struct TGData {
        std::vector<glm::vec3> baseVelocities;
        LIC<3>* varCurve;
        std::vector<int> pointIndexes;
        std::vector<float> minValues; // tama�o D*pointIndexes.size()

        float descentRate;
        int maxIterations;
    };

    class IKRig;
    class IKRigConfig;
    class TrajectoryGenerator {
        friend class IKNavigationComponent;
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        std::vector<ChainIndex> m_ikChains;
        GradientDescent<TGData> m_gradientDescent;
        TGData m_tgData;
        void generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateStaticTrajectory(EETrajectory baseTrajectory,
            ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateDynamicTrajectory(EETrajectory baseTrajectory,
            ChainIndex ikChain, IKRigConfig* config,
            float xyMovementRotAngle,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateHipTrajectory(IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);

        float calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float targetCurvesTime_rep,
            float originalCurvesTime_anim);
        glm::vec3 calcStrideStartingPoint(float supportHeight, glm::vec3 referencePoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        std::vector<glm::vec3> calcStrideData(float supportHeight, glm::vec3 startingPoint, float targetDistance,
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains);
        TrajectoryGenerator() = default;
        void generateNewTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        glm::vec3 getModelSpaceEETargetPos(ChainIndex ikChain);
        glm::vec3 getHipTargetPos();


    };

    
}





#endif