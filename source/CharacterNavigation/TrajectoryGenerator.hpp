#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"
#include "ParametricCurves.hpp"
#include "TrajectoryGeneratorBase.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;
    typedef int FrameIndex;

    struct TGData {
        std::vector<glm::vec3> baseVelocitiesR;
        std::vector<glm::vec3> baseVelocitiesL;
        float alphaValue = 0.8;
        float betaValue = 0.2;
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
        friend class IKRigController;
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        std::vector<ChainIndex> m_ikChains;
        GradientDescent<TGData> m_gradientDescent;
        TGData m_tgData;
        void generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateStaticTrajectory(EETrajectory baseTrajectory, ChainIndex ikChain, 
            IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateDynamicTrajectory(EETrajectory baseTrajectory,
            ChainIndex ikChain, IKRigConfig* config,
            float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateHipTrajectory(IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);

        float calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float targetCurvesTime_rep,
            float originalCurvesTime_extendedAnim);
        glm::vec3 calcStrideStartingPoint(float supportHeight, glm::vec3 referencePoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        std::vector<glm::vec3> calcStrideData(float supportHeight, glm::vec3 startingPoint, float targetDistance,
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains);
        TrajectoryGenerator() = default;
        void init();
        void generateNewTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        std::vector<ChainIndex> getIKChains() { return m_ikChains; }
        void setIKChains(std::vector<ChainIndex> ikChains) { m_ikChains = ikChains; }
    };

    
}





#endif