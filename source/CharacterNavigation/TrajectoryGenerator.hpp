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
        float alphaValue;
        float betaValue;
        LIC<3>* varCurve;
        LIC<3> baseCurve;
        std::vector<int> pointIndexes;
        std::vector<float> minValues; // tamano D*pointIndexes.size()
        float descentRate;
        float targetPosDelta;
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
        void generateFixedTrajectory(glm::vec2 basePos,
            glm::vec2 timeRange, float supportHeight,
            ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        LIC<3> calcSimplifiedTrajectoryExtension(float reproductionTime1, float reproductionTime2, 
            bool extendEnd, glm::vec3 anchorPoint,
            ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateHipTrajectory(IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);

		float calcHipAdjustedHeight(glm::vec2 basePoint, float targetCurvesTime_rep,
			float originalCurvesTime_extendedAnim, IKRigConfig* config,
			ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager);
        glm::vec3 calcStrideStartingPoint(float supportHeight, glm::vec2 xyReferencePoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        std::vector<glm::vec3> calcStrideData(float supportHeightStart, float supportHeightEnd, 
            glm::vec3 startingPoint, float targetDistance,
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