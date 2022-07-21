#pragma once
#ifndef TRAJECTORYGENERATOR_HPP
#define TRAJECTORYGENERATOR_HPP

#include "EnvironmentData.hpp"
#include "GradientDescent.hpp"
#include "ParametricCurves.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;
    typedef int FrameIndex;

    enum class TrajectoryType {
        STATIC,
        DYNAMIC
    };

    class HipGlobalTrajectoryData {
        friend class IKRigController;
        // Angulos de rotacion originales
        LIC<1> m_originalRotationAngles;
        // Ejes de rotacion originales
        LIC<3> m_originalRotationAxes;
        // Traslaciones originales
        LIC<3> m_originalTranslations;
        // Direccion original de la cadera
        glm::vec2 m_originalFrontVector;
        LIC<1> m_targetRotationAngles;
        LIC<3> m_targetRotationAxes;
        LIC<3> m_targetTranslations;
        std::vector<glm::vec3> m_savedTranslations;
        std::vector<float> m_savedRotationAngles;
        std::vector<glm::vec3> m_savedRotationAxes;
    public:
        LIC<1> getOriginalRotationAngles() { return m_originalRotationAngles; }
        LIC<3> getOriginalRotationAxes() { return m_originalRotationAxes; }
        LIC<3> getOriginalTranslations() { return m_originalTranslations; }
        glm::vec2 getOriginalFrontVector() { return m_originalFrontVector; }
        glm::vec3 getSavedTranslation(FrameIndex frame) { return m_savedTranslations[frame]; }
        float getSavedRotationAngle(FrameIndex frame) { return m_savedRotationAngles[frame]; }
        glm::vec3 getSavedRotationAxis(FrameIndex frame) { return m_savedRotationAxes[frame]; }
        LIC<1> getTargetRotationAngles() { return m_targetRotationAngles; }
        LIC<3> getTargetRotationAxes() { return m_targetRotationAxes; }
        LIC<3> getTargetTranslations() { return m_targetTranslations; }
        void setTargetRotationAngles(LIC<1> targetRotationAngles) { m_targetRotationAngles = targetRotationAngles; }
        void setTargetRotationAxes(LIC<3> targetRotationAxes) { m_targetRotationAxes = targetRotationAxes; }
        void setTargetTranslations(LIC<3> targetTranslations) { m_targetTranslations = targetTranslations; }
    };
    class EETrajectory {
        friend class IKRigController;
        friend class EETrajectoryData;
        LIC<3> m_curve;
        TrajectoryType m_trajectoryType;
        // Indice del punto de la curva que tiene el tiempo en el que ocurre la
        // de maxima altura de la cadera
        int m_hipMaxAltitudeIndex = -1;
    public:
        EETrajectory() = default;
        EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType);
        bool isDynamic() { return m_trajectoryType == TrajectoryType::DYNAMIC; }
        LIC<3>& getEECurve() { return m_curve; }
        int getHipMaxAltitudeIndex() {
            MONA_ASSERT(m_hipMaxAltitudeIndex != -1, "EETrajectory: Accessing unitialized value.");
            return m_hipMaxAltitudeIndex;
        }
    };
    class EEGlobalTrajectoryData {
        friend class IKRigController;
        // Trayectoria original del ee asociado a una ikChain. Descompuesta en sub trayectorias (estaticas y dinamicas)
        std::vector<EETrajectory> m_originalSubTrajectories;
        // Trayectoria objetivo generada
        EETrajectory m_targetTrajectory;;
        // Altura base en cada frame, considerando los valores en los frames de soporte
        std::vector<float> m_supportHeights;
        // Posiciones guardadas calculadas para frames previos con IK
        std::vector<glm::vec3> m_savedPositions;
    public:
        EETrajectory getSubTrajectory(float animationTime);
        glm::vec3 getSavedPosition(FrameIndex frame) { return m_savedPositions[frame]; }
        float getSupportHeight(FrameIndex frame) { return m_supportHeights[frame]; }
        EETrajectory& getTargetTrajectory() { return m_targetTrajectory; }
        void setTargetTrajectory(LIC<3> trajectory, TrajectoryType trajectoryType) { m_targetTrajectory = EETrajectory(trajectory, trajectoryType); }

    };

    struct TGData {
        std::vector<glm::vec3> baseVelocities;
        LIC<3>* varCurve;
        std::vector<int> pointIndexes;
        std::vector<float> minValues; // tamaño D*pointIndexes.size()

        float descentRate;
        int maxIterations;
    };

    class IKRig;
    class IKRigConfig;
    class TrajectoryGenerator {
        friend class IKNavigationComponent;
        friend class IKRigController;
    private:
        IKRig* m_ikRig;
        EnvironmentData m_environmentData;
        std::vector<ChainIndex> m_ikChains;
        GradientDescent<TGData> m_gradientDescent;
        TGData m_tgData;
        void generateEETrajectory(ChainIndex ikChain, IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateStaticTrajectory(EETrajectory baseTrajectory,
            ChainIndex ikChain, IKRigConfig* config,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateDynamicTrajectory(EETrajectory baseTrajectory,
            ChainIndex ikChain, IKRigConfig* config,
            float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        void generateHipTrajectory(IKRigConfig* config, float xyMovementRotAngle,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);

        float calcHipAdjustedHeight(IKRigConfig* config, glm::vec2 basePoint, float targetCurvesTime_rep,
            float originalCurvesTime_anim);
        glm::vec3 calcStrideStartingPoint(float supportHeight, glm::vec3 referencePoint, float targetDistance, 
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        std::vector<glm::vec3> calcStrideData(float supportHeight, glm::vec3 startingPoint, float targetDistance,
            glm::vec2 targetDirection, int stepNum,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
    public:
        TrajectoryGenerator(IKRig* ikRig, std::vector<ChainIndex> ikChains);
        TrajectoryGenerator() = default;
        void generateNewTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);
        std::vector<ChainIndex> getIKChains() { return m_ikChains; }
        void setIKChains(std::vector<ChainIndex> ikChains) { m_ikChains = ikChains; }
    };

    
}





#endif