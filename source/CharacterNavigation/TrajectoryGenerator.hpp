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

    class IKRig;
    class IKRigConfig;
    class TrajectoryGenerator {
        friend class IKNavigationComponent;
        friend class IKRigController;
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        void generateEETrajectory(ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateFixedTrajectory(glm::vec2 basePos,
            glm::vec2 timeRange, int baseCurveID, float supportHeight,
            ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateHipTrajectory(IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
		float calcHipAdjustedHeight(std::pair<EEGlobalTrajectoryData*, EEGlobalTrajectoryData*> oppositeTrajectories, 
            float targetCurvesTime_rep,
            float originalCurvesTime_extendedAnim, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        bool calcStrideStartingPoint(float supportHeightStart, glm::vec2 xyReferencePoint, float targetDistance, 
            glm::vec2 targetDirection, glm::vec3& outStrideStartPoint,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        bool calcStrideFinalPoint(float supportHeightStart, float supportHeightEnd,
            glm::vec3 startingPoint, float targetDistance,
            glm::vec2 targetDirection, glm::vec3& outStrideFinalPoint,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
		static void buildHipTrajectory(IKRigConfig* config, std::vector<glm::vec3> const& hipGlobalPositions);
		static void buildEETrajectories(IKRigConfig* config,
			std::vector<std::vector<bool>> supportFramesPerChain,
			std::vector<std::vector<glm::vec3>> globalPositionsPerChain,
            std::vector<ChainIndex> oppositePerChain);
    public:
        TrajectoryGenerator(IKRig* ikRig);
        TrajectoryGenerator() = default;
        void generateNewTrajectories(AnimationIndex animIndex,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
    };

    
}





#endif