#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"

namespace Mona{

    typedef int ChainIndex;
    struct TGData {

    };

    class IKRig;
    class TrajectoryGenerator {
        friend class IKNavigationComponent;
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        GradientDescent<TGData> m_gradientDescent;
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> regularChains, ChainIndex hipChain);
        TrajectoryGenerator() = default;
        void setIKChains(std::vector<ChainIndex> regularChains, ChainIndex hipChain);


    };

    
}





#endif