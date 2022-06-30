#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"
#include "ParametricCurves.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;
    struct TGData {
        DiscreteCurve* varCurve;
        std::vector<glm::vec3> baseVelocities;
        std::vector<int> pointIndexes;
    };

    class IKRig;
    class TrajectoryGenerator {
        friend class IKNavigationComponent;
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        GradientDescent<TGData> m_gradientDescent;
        TGData m_tgData;
        ChainIndex m_hipChain;
        BezierSpline generateRegularTrajectory(ChainIndex regularChain, AnimationIndex animIndex);
        BezierSpline generateHipTrajectory();
    public:
        TrajectoryGenerator(IKRig* ikRig, ChainIndex hipChain);
        TrajectoryGenerator() = default;
        void setHipKChain(ChainIndex hipChain);
        std::vector<std::pair<ChainIndex, BezierSpline>> setNewTrajectories(AnimationIndex animIndex, std::vector<ChainIndex> regularChains);


    };

    
}





#endif