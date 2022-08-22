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
        friend class DebugDrawingSystem_ikNav;
        friend class TrajectoryGenerator;
        // Angulos de rotacion originales
        LIC<1> m_originalRotationAngles;
        // Ejes de rotacion originales
        LIC<3> m_originalRotationAxes;
        // Traslaciones originales
        LIC<3> m_originalTranslations;
        LIC<1> m_targetRotationAngles;
        LIC<3> m_targetRotationAxes;
        LIC<3> m_targetTranslations;
        std::vector<glm::vec3> m_savedTranslations;
        std::vector<bool> m_savedDataValid;
        IKRigConfig* m_config;
		template <int D>
        LIC<D> sampleOriginaCurve(float initialExtendedAnimTime, float finalExtendedAnimTime,
            LIC<D>& originalCurve);
    public:
        LIC<1> sampleOriginalRotationAngles(float initialExtendedAnimTime, float finalExtendedAnimTime);
        LIC<3> sampleOriginalRotationAxes(float initialExtendedAnimTime, float finalExtendedAnimTime);
        LIC<3> sampleOriginalTranslations(float initialExtendedAnimTime, float finalExtendedAnimTime);
        glm::vec3 getSavedTranslation(FrameIndex frame) { return m_savedTranslations[frame]; }
        bool isSavedDataValid(FrameIndex frame) { return m_savedDataValid[frame]; }
        LIC<1> getTargetRotationAngles() { return m_targetRotationAngles; }
        LIC<3> getTargetRotationAxes() { return m_targetRotationAxes; }
        LIC<3> getTargetTranslations() { return m_targetTranslations; }
        glm::fquat getTargetRotation(float reproductionTime);
        glm::vec3 getTargetTranslation(float reproductionTime);
        void setTargetRotationAngles(LIC<1> targetRotationAngles) { m_targetRotationAngles = targetRotationAngles; }
        void setTargetRotationAxes(LIC<3> targetRotationAxes) { m_targetRotationAxes = targetRotationAxes; }
        void setTargetTranslations(LIC<3> targetTranslations) { m_targetTranslations = targetTranslations; }
        void init(IKRigConfig* config);
    };

    class EETrajectory {
        friend class IKRigController;
        friend class EETrajectoryData;
        friend class TrajectoryGenerator;
        LIC<3> m_curve;
        LIC<3> m_altCurve;
        TrajectoryType m_trajectoryType;
        // Indice del punto de la curva que tiene el tiempo en el que ocurre la
        // de maxima altura de la cadera
        float m_hipMaxAltitudeTime = std::numeric_limits<float>::lowest();
        int m_subTrajectoryID = -1;
    public:
        EETrajectory() = default;
        EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID = -1, LIC<3> altCurve=LIC<3>());
        bool isDynamic() { return m_trajectoryType == TrajectoryType::DYNAMIC; }
        LIC<3>& getEECurve() { return m_curve; }
        LIC<3>& getAltCurve() { return m_altCurve; }
        float getHipMaxAltitudeTime() {
            MONA_ASSERT(m_hipMaxAltitudeTime != std::numeric_limits<float>::lowest(), "EETrajectory: Accessing unitialized value.");
            return m_hipMaxAltitudeTime;
        }
        int getSubTrajectoryID() {
			return m_subTrajectoryID;
        }
    };
    class EEGlobalTrajectoryData {
        friend class IKRigController;
        friend class TrajectoryGenerator;
        friend class DebugDrawingSystem_ikNav;
        // Trayectoria original del ee asociado a una ikChain. Descompuesta en sub trayectorias (estaticas y dinamicas)
        std::vector<EETrajectory> m_originalSubTrajectories;
        // Trayectoria objetivo generada
        EETrajectory m_targetTrajectory;;
        // Altura base en cada frame, considerando los valores en los frames de soporte
        std::vector<float> m_supportHeights;
        // Posiciones guardadas calculadas para frames previos con IK
        std::vector<glm::vec3> m_savedPositions;
        std::vector<bool> m_savedDataValid;
        IKRigConfig* m_config;
    public:
        LIC<3> sampleExtendedSubTrajectory(float animationTime, float duration);
        EETrajectory getSubTrajectory(float animationTime);
        EETrajectory getSubTrajectoryByID(int subTrajectoryID);
        int getSubTrajectoryNum() { return m_originalSubTrajectories.size(); }
        glm::vec3 getSavedPosition(FrameIndex frame) { return m_savedPositions[frame]; }
        bool isSavedDataValid(FrameIndex frame) { return m_savedDataValid[frame]; }
        float getSupportHeight(FrameIndex frame) { return m_supportHeights[frame]; }
        EETrajectory& getTargetTrajectory() { return m_targetTrajectory; }
        void setTargetTrajectory(LIC<3> curve, TrajectoryType trajectoryType, int subTrajectoryID, LIC<3> altCurve = LIC<3>()) { 
            m_targetTrajectory = EETrajectory(curve, trajectoryType, subTrajectoryID, altCurve); }
        void init(IKRigConfig* config);
    };
    
}





#endif