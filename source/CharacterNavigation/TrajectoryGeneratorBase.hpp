#pragma once
#ifndef TRAJECTORYGENERATORBASE_HPP
#define TRAJECTORYGENERATORBASE_HPP

#include "ParametricCurves.hpp"

namespace Mona{

    typedef int ChainIndex;
    typedef int AnimationIndex;
    typedef int FrameIndex;

    enum class TrajectoryType {
        STATIC,
        DYNAMIC
    };

    class IKRigConfig;
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
        IKRigConfig* m_config;
    public:
        LIC<1> sampleOriginalRotationAngles(float initialAnimTime, float finalAnimTime);
        LIC<3> sampleOriginalRotationAxes(float initialAnimTime, float finalAnimTime);
        LIC<3> sampleOriginalTranslations(float initialAnimTime, float finalAnimTime);
        glm::vec2 getOriginalFrontVector() { return m_originalFrontVector; }
        glm::vec3 getSavedTranslation(FrameIndex frame) { return m_savedTranslations[frame]; }
        float getSavedRotationAngle(FrameIndex frame) { return m_savedRotationAngles[frame]; }
        glm::vec3 getSavedRotationAxis(FrameIndex frame) { return m_savedRotationAxes[frame]; }
        LIC<1> getTargetRotationAngles() { return m_targetRotationAngles; }
        LIC<3> getTargetRotationAxes() { return m_targetRotationAxes; }
        LIC<3> getTargetTranslations() { return m_targetTranslations; }
        glm::fquat getTargetRotation(float reproductionTime);
        glm::vec3 getTargetTranslation(float reproductionTime);
        void setTargetRotationAngles(LIC<1> targetRotationAngles) { m_targetRotationAngles = targetRotationAngles; }
        void setTargetRotationAxes(LIC<3> targetRotationAxes) { m_targetRotationAxes = targetRotationAxes; }
        void setTargetTranslations(LIC<3> targetTranslations) { m_targetTranslations = targetTranslations; }
        void init(int frameNum, IKRigConfig* config);
    };

    class EETrajectory {
        friend class IKRigController;
        friend class EETrajectoryData;
        LIC<3> m_curve;
        TrajectoryType m_trajectoryType;
        // Indice del punto de la curva que tiene el tiempo en el que ocurre la
        // de maxima altura de la cadera
        int m_hipMaxAltitudeIndex = -1;
        int m_subTrajectoryID = -1;
    public:
        EETrajectory() = default;
        EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID = -1);
        bool isDynamic() { return m_trajectoryType == TrajectoryType::DYNAMIC; }
        LIC<3>& getEECurve() { return m_curve; }
        int getHipMaxAltitudeIndex() {
            MONA_ASSERT(m_hipMaxAltitudeIndex != -1, "EETrajectory: Accessing unitialized value.");
            return m_hipMaxAltitudeIndex;
        }
        int getSubTrajectoryID() {
			MONA_ASSERT(m_subTrajectoryID != -1, "EETrajectory: Accessing unitialized value.");
			return m_subTrajectoryID;
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
        EETrajectory getSubTrajectoryByID(int subTrajectoryID);
        int getSubTrajectoryNum() { return m_originalSubTrajectories.size(); }
        glm::vec3 getSavedPosition(FrameIndex frame) { return m_savedPositions[frame]; }
        float getSupportHeight(FrameIndex frame) { return m_supportHeights[frame]; }
        EETrajectory& getTargetTrajectory() { return m_targetTrajectory; }
        void setTargetTrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID) { 
            m_targetTrajectory = EETrajectory(trajectory, trajectoryType, subTrajectoryID); }
        void init(int frameNum);
    };
    
}





#endif