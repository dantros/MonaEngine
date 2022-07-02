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
    };

    class IKRig;
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
        GradientDescent<TGData<1>> m_gradientDescent_dim1;
        TGData<1> m_tgData_dim1;
        GradientDescent<TGData<3>> m_gradientDescent_dim3;
        TGData<3> m_tgData_dim3;
        std::pair<TrajectoryGenerator::TrajectoryType, LIC<3>> generateRegularTrajectory(ChainIndex regularChain, AnimationIndex animIndex);
        LIC<3> generateHipTrajectory();
    public:
        TrajectoryGenerator(IKRig* ikRig);
        TrajectoryGenerator() = default;
        void setNewTrajectories(AnimationIndex animIndex, std::vector<ChainIndex> regularChains);


    };

    
}





#endif