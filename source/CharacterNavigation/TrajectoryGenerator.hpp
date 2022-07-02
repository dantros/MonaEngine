#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"
#include "ParametricCurves.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;

    template <typename T>
    struct TGData {
        LIC<T>* baseCurve;
        LIC<T>* varCurve;
        std::vector<T> baseVelocities;
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
        GradientDescent<TGData<glm::vec1>> m_gradientDescent_dim1;
        TGData<glm::vec1> m_tgData_dim1;
        GradientDescent<TGData<glm::vec3>> m_gradientDescent_dim3;
        TGData<glm::vec3> m_tgData_dim3;
        std::pair<TrajectoryType, LIC<glm::vec3>> generateRegularTrajectory(ChainIndex regularChain, AnimationIndex animIndex);
        LIC<glm::vec3> generateHipTrajectory();
    public:
        TrajectoryGenerator(IKRig* ikRig);
        TrajectoryGenerator() = default;
        void setNewTrajectories(AnimationIndex animIndex, std::vector<ChainIndex> regularChains);


    };

    
}





#endif