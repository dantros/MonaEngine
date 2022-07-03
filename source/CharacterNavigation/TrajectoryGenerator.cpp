#include "TrajectoryGenerator.hpp"
#include "IKRig.hpp"
#include "../Core/GlmUtils.hpp"

namespace Mona{

    // funciones para descenso de gradiente
    template <int D>
    std::function<float(const std::vector<float>&, TGData<D>*)> term1Function =
        [](const std::vector<float>& varPCoord, TGData<D>* dataPtr)->float {
        float result = 0;
        for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
            int pIndex = dataPtr->pointIndexes[i];
            float tVal = dataPtr->varCurve->getTValues()[pIndex];
            result += glm::distance2(dataPtr->varCurve->getLeftHandVelocity(tVal),
                dataPtr->baseCurve->getLeftHandVelocity(tVal));
        }
        return result;
    };

    template <int D>
    std::function<float(const std::vector<float>&, int, TGData<D>*)> term1PartialDerivativeFunction =
        [](const std::vector<float>& varPCoord, int varIndex, TGData<D>* dataPtr)->float {
        int pIndex = dataPtr->pointIndexes[varIndex / D];
        int coordIndex = varIndex % D;
        float tVal = dataPtr->varCurve->getTValues()[pIndex];
        float result = pIndex != 0 ?
            2 * (dataPtr->varCurve->getLeftHandVelocity(tVal)[coordIndex] - dataPtr->baseCurve->getLeftHandVelocity(tVal)[coordIndex])
            / (dataPtr->varCurve->getTValues()[pIndex] - dataPtr->varCurve->getTValues()[pIndex - 1]) : 0;

        return result;
    };

    template <int D>
    std::function<void(std::vector<float>&, TGData<D>*)>  postDescentStepCustomBehaviour = 
        [](std::vector<float>& varPCoord, TGData<D>* dataPtr)->void {
        glm::vec<D, float> newPos;
        for (int i = 0; i < D; i++) {
            for (int j = 0; j < D; j++) {
                newPos[j] = varPCoord[i * D + j];
            }
            int pIndex = dataPtr->pointIndexes[i];
            newPos = glm::max<glm::vec<D, float>>(dataPtr->minValues[i], newPos);
            dataPtr->varCurve->setCurvePoint(pIndex, newPos);
        }
    };


    TrajectoryGenerator::TrajectoryGenerator(IKRig* ikRig) {
        m_ikRig = ikRig;
       
        // descenso para angulos (dim 1)
        FunctionTerm<TGData<1>> dim1Term(term1Function<1>, term1PartialDerivativeFunction<1>);
        m_gradientDescent_dim1 = GradientDescent<TGData<1>>({ dim1Term }, 0, &m_tgData_dim1, postDescentStepCustomBehaviour<1>);

        // descenso para posiciones (dim 3)
        FunctionTerm<TGData<3>> dim3Term(term1Function<3>, term1PartialDerivativeFunction<3>);
        m_gradientDescent_dim3 = GradientDescent<TGData<3>>({ dim3Term }, 0, &m_tgData_dim3, postDescentStepCustomBehaviour<3>);
    }

    void setNewTrajectories(AnimationIndex animIndex, std::vector<ChainIndex> regularChains) {
        // las trayectorias anteriores siempre deben llegar hasta el currentFrame
        // se necesitan para cada ee su posicion actual y la curva base, ambos en espacio global
        // se usa "animationTime" que corresponde al tiempo de la aplicacion modificado con el playRate (-- distinto a samplingTime--)
    }

    glm::vec3 removeHipMotion(glm::vec3 pseudoModelSpacePos, float animationLoopTime, IKRigConfig* config) {
        auto hipTrData = config->getHipTrajectoryData();
        float pseudoMSHipRotAngle = hipTrData->originalRotationAngles.evalCurve(animationLoopTime)[0];
        glm::vec3 pseudoMSHipRotAxis = hipTrData->originalRotationAxes.evalCurve(animationLoopTime);
        glm::fquat pseudoMSHipRot = glm::fquat(pseudoMSHipRotAngle, pseudoMSHipRotAxis);
        glm::vec3 pseudoMSHipTr = hipTrData->originalGlblTranslations.evalCurve(animationLoopTime);
        glm::mat4 hipMotion = glmUtils::translationToMat4(pseudoMSHipTr)*
            glmUtils::rotationToMat4(pseudoMSHipRot);
        return glm::inverse(hipMotion) * glm::vec4(pseudoModelSpacePos, 1);

    }

    std::pair<TrajectoryGenerator::TrajectoryType, LIC<3>> TrajectoryGenerator::generateRegularTrajectory(ChainIndex regularChain, AnimationIndex animIndex) {
        IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
        EETrajectoryData* trData = config->getTrajectoryData(regularChain);
        HipTrajectoryData* hipTrData = config->getHipTrajectoryData();
        FrameIndex nextFrameIndex = config->getNextFrameIndex();
        FrameIndex currentFrameIndex = config->getCurrentFrameIndex();

        // chequemos que tipo de trayectoria hay que crear (estatica o dinamica)
        // si es estatica
        if (trData->supportFrames[nextFrameIndex]) {
            float initialTime_loop = config->getTimeStamps()[currentFrameIndex];
            float initialTime = config->getAnimationTime(initialTime_loop);
            glm::vec3 initialPos;
            // chequear si hay una curva previa generada
            if (trData->targetGlblTrajectory.inTRange(initialTime)) {
                initialPos = trData->targetGlblTrajectory.evalCurve(initialTime);
            }
            else {
                // Debemos llevar la posicion del ee en pseudo model space a model space y luego a global. luego de vuelta a model space
                glm::vec3 pseudoMSPos = trData->originalGlblTrajectory.evalCurve(initialTime_loop);
                glm::vec3 msPos = removeHipMotion(pseudoMSPos, initialTime_loop, config);
            }
            float finalTime = config->getTimeStamps()[nextFrameIndex];
            for (FrameIndex f = nextFrameIndex + 1; f < config->getTimeStamps().size(); f++) {
                if (trData->supportFrames[f]) { finalTime = config->getTimeStamps()[f]; }
                else { break; }
            }
            finalTime = config->getAnimationTime(finalTime);
            std::vector<float> tValues = { initialTime, finalTime };
            std::vector<glm::vec3> splinePoints = { initialPos, initialPos };
            return std::pair<TrajectoryType, LIC<3>>(TrajectoryType::STATIC ,
                LIC(splinePoints, tValues));
        } // si es dinamica
        else {

        }
        
    }




    
}