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

    template <int D>
    struct TGData {
        std::vector<glm::vec<D,float>> baseVelocities;
        LIC<D>* varCurve;
        std::vector<int> pointIndexes;
        std::vector<float> minValues; // tamaño D*pointIndexes.size()

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
        GradientDescent<TGData<1>> m_gradientDescent_dim1;
        TGData<1> m_tgData_dim1;
        GradientDescent<TGData<3>> m_gradientDescent_dim3;
        TGData<3> m_tgData_dim3;
        std::pair<FrameIndex, FrameIndex> calcTrajectoryFrameRange(IKRigConfig* config, ChainIndex ikChain, TrajectoryType trajectoryType);
        void generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, 
            glm::vec3 globalEEPos, float rotationAngle,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateStaticTrajectory(ChainIndex ikChain, IKRigConfig* config,
            glm::vec3 globalEEPos,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateDynamicTrajectory(ChainIndex ikChain, IKRigConfig* config,
            glm::vec3 globalEEPos, float rotationAngle,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateHipTrajectory();

        glm::vec3 calcStrideStartingPoint(glm::vec3 referencePoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        std::vector<glm::vec3> calcStrideData(glm::vec3 startingPoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains);
        TrajectoryGenerator() = default;
        void generateTrajectories(AnimationIndex animIndex);
        glm::vec3 getModelSpaceEETargetPos(ChainIndex ikChain);
        glm::vec3 getHipTargetPos();


    };

    
}





#endif