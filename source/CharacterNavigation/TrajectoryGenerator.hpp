#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"
#include "ParametricCurves.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;

    template <int D>
    struct TGData {
        LIC<D>* baseCurve;
        LIC<D>* varCurve;
        std::vector<int> pointIndexes;
        std::vector<float> minValues; // tamaño D*pointIndexes.size()
    };

    class IKRig;
    class IKRigConfig;
    class TrajectoryGenerator {
        friend class IKNavigationComponent;
    public:
        enum class TrajectoryType {
            STATIC,
            DYNAMIC
        };
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        std::vector<ChainIndex> m_ikChains;
        GradientDescent<TGData<1>> m_gradientDescent_dim1;
        TGData<1> m_tgData_dim1;
        GradientDescent<TGData<3>> m_gradientDescent_dim3;
        TGData<3> m_tgData_dim3;
        TrajectoryType generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, 
            glm::vec3 globalEEPos,
            ComponentManager<TransformComponent>* transformManager,
            ComponentManager<StaticMeshComponent>* staticMeshManager);
        void generateHipTrajectory();
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains);
        TrajectoryGenerator() = default;
        void generateTrajectories(AnimationIndex animIndex);


    };

    
}





#endif