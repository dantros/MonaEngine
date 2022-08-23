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
        GradientDescent<TGData> m_gradientDescent;
        TGData m_tgData;
        void generateEETrajectory(ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateFixedTrajectory(glm::vec2 basePos,
            glm::vec2 timeRange, float supportHeight,
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
        glm::vec3 calcStrideStartingPoint(float supportHeight, glm::vec2 xyReferencePoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        std::vector<glm::vec3> calcStrideData(float supportHeightStart, float supportHeightEnd, 
            glm::vec3 startingPoint, float targetDistance,
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
		static void buildHipTrajectory(IKRigConfig* config, std::vector<glm::mat4> const& hipGlobalTransforms, float minDistance, float floorZ);
		static void buildEETrajectories(IKRigConfig* config,
			std::vector<std::vector<bool>> supportFramesPerChain,
			std::vector<std::vector<glm::vec3>> globalPositionsPerChain);
    public:
        TrajectoryGenerator(IKRig* ikRig);
        TrajectoryGenerator() = default;
        void init();
        void generateNewTrajectories(AnimationIndex animIndex,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
    };

    
}





#endif