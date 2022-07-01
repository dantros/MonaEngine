#include "TrajectoryGenerator.hpp"
#include "IKRig.hpp"

namespace Mona{

    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig) {
        m_ikRig = ikRig;
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

    std::vector<std::pair<ChainIndex, BezierSpline>> setNewTrajectories(AnimationIndex animIndex, std::vector<ChainIndex> regularChains) {
        // las trayectorias anteriores siempre deben llegar hasta el currentFrame
        // se necesitan para cada ee su posicion actual y la curva base, ambos en espacio global
        // se usa "animationTime" que corresponde al tiempo de la aplicacion modificado con el playRate (-- distinto a samplingTime--)
    }

    std::pair<TrajectoryGenerator::TrajectoryType, BezierSpline> TrajectoryGenerator::generateRegularTrajectory(ChainIndex regularChain, AnimationIndex animIndex) {
        IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
        EETrajectoryData* trData = config->getTrajectoryData(regularChain);
        EETrajectoryData* hipTrData = config->getTrajectoryData(m_hipChain);
        FrameIndex nextFrameIndex = config->getNextFrameIndex();

        // chequemos que tipo de trayectoria hay que crear (estatica o dinamica)
        // si es estatica
        if (trData->eeSupportFrames[nextFrameIndex]) {
            float initialTime = config->getCurrentFrameIndex();
            glm::vec3 initialPos;
            // chequear si hay una curva previa generada
            if (trData->eeTargetTrajectory.inTRange(initialTime)) {
                initialPos = trData->eeTargetTrajectory.evalSpline(initialTime);
            }
            else {
                // Debemos llevar la posicion del ee en pseudo model space a model space y luego a global. luego de vuelta a model space
                glm::vec3 pseudoMSPos = trData->eeBaseTrajectory.evalSpline(initialTime);
                glm::vec3 pseudoMSHipPos = hipTrData->eeBaseTrajectory.evalSpline(initialTime);
                glm::vec3 msPos = pseudoMSPos - pseudoMSHipPos;
            }
            float finalTime = config->getTimeStamps()[nextFrameIndex];
            for (FrameIndex f = nextFrameIndex + 1; f < config->getTimeStamps().size(); f++) {
                if (trData->eeSupportFrames[f]) { finalTime = config->getTimeStamps()[f]; }
                else { break; }
            }
            finalTime = config->getAnimationTime(finalTime);
            std::vector<float> tValues = { initialTime, finalTime };
            std::vector<glm::vec3> splinePoints = { initialPos, initialPos };
            return std::pair<TrajectoryType, BezierSpline>(TrajectoryType::STATIC ,
                BezierSpline(splinePoints, tValues, BezierSpline::Order::LINEAR));
        } // si es dinamica
        else {

        }
        
    }




    
}