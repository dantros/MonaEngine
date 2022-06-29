#include "TrajectoryGenerator.hpp"

namespace Mona{

    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> regularChains, ChainIndex hipChain) {
        m_ikRig = ikRig;
        m_regularChains = regularChains;
        m_hipChain = hipChain;
        //creamos terminos para el descenso de gradiente
        std::function<float(const std::vector<float>&, TGData*)> term1Function =
            [](const std::vector<float>& varPCoord, TGData* dataPtr)->float {
            float result = 0;
            for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
                int pIndex = dataPtr->pointIndexes[i];
                result += glm::distance2(dataPtr->varCurve->getVelocity(pIndex), dataPtr->baseVelocities[pIndex]);
            }
            return result;
        };

        std::function<float(const std::vector<float>&, int, TGData*)> term1PartialDerivativeFunction =
            [](const std::vector<float>& varPCoord, int varIndex, TGData* dataPtr)->float {
            int pIndex = dataPtr->pointIndexes[varIndex / 3];
            int coordIndex = varIndex % 3;
            float result = pIndex != 0 ? 
                2 * (dataPtr->varCurve->getVelocity(pIndex)[coordIndex] - dataPtr->baseVelocities[pIndex][coordIndex])
                / (dataPtr->varCurve->getTValues()[pIndex] - dataPtr->varCurve->getTValues()[pIndex-1])   : 0;

            return result;
        };

        FunctionTerm<TGData> term1(term1Function, term1PartialDerivativeFunction);

        std::function<void(std::vector<float>&, TGData*)>  postDescentStepCustomBehaviour = [](std::vector<float>& varPCoord, TGData* dataPtr)->void {
            glm::vec3 newPos;
            for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
                for (int j = 0; j < 3; j++) {
                    newPos[j] = varPCoord[i * 3 + j];
                }
                int pIndex = dataPtr->pointIndexes[i];
                dataPtr->varCurve->setCurvePoint(pIndex, newPos);
            }
        };

        auto terms = std::vector<FunctionTerm<TGData>>({ term1 });
        m_gradientDescent = GradientDescent<TGData>(terms, 0, &m_tgData, postDescentStepCustomBehaviour);
    }


    void TrajectoryGenerator::setIKChains(std::vector<ChainIndex> regularChains, ChainIndex hipChain) {
        m_regularChains = regularChains;
        m_hipChain = hipChain;
    }




    
}