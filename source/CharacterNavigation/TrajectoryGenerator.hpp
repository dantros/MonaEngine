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
        std::vector<ChainIndex> m_regularChains;
        ChainIndex m_hipChain;
        BezierSpline generateStatic();
        std::vector<std::pair<ChainIndex, BezierSpline>> generateDynamic();
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> regularChains, ChainIndex hipChain);
        TrajectoryGenerator() = default;
        void setIKChains(std::vector<ChainIndex> regularChains, ChainIndex hipChain);
        std::vector<std::pair<ChainIndex, BezierSpline>> generateTrajectories(AnimationIndex animIndex);


    };

    
}





#endif