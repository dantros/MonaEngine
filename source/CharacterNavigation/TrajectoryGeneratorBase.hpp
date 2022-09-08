#pragma once
#ifndef TRAJECTORYGENERATORBASE_HPP
#define TRAJECTORYGENERATORBASE_HPP

#include "ParametricCurves.hpp"
#include "GradientDescent.hpp"
#include "EnvironmentData.hpp"

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
        // Traslaciones originales
        LIC<3> m_originalPositions;
        LIC<3> m_targetPositions;
        LIC<3> m_savedPositions;
        bool m_motionInitialized = false;
        IKRigConfig* m_config;
		template <int D>
        LIC<D> sampleOriginaCurve(float initialExtendedAnimTime, float finalExtendedAnimTime,
            LIC<D>& originalCurve);
    public:
        LIC<3> sampleOriginalPositions(float initialExtendedAnimTime, float finalExtendedAnimTime);
        LIC<3> const& getSavedPositions() { return m_savedPositions; }
        bool motionInitialized() { return m_motionInitialized; }
        LIC<3> getTargetPositions() { return m_targetPositions; }
        void setTargetPositions(LIC<3> targetPositions) { m_targetPositions = targetPositions; }
        void init(IKRigConfig* config);
        void refresh();
    };

    class EETrajectory {
        friend class IKRigController;
        friend class EETrajectoryData;
        friend class EEGlobalTrajectoryData;
        friend class TrajectoryGenerator;
        LIC<3> m_curve;
        TrajectoryType m_trajectoryType;
        int m_subTrajectoryID = -1;
    public:
        EETrajectory() = default;
        EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID = -1);
        bool isDynamic() { return m_trajectoryType == TrajectoryType::DYNAMIC; }
        LIC<3>& getEECurve() { return m_curve; }
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
        LIC<3> m_savedPositions;
        bool m_motionInitialized = false;
        IKRigConfig* m_config;
        EEGlobalTrajectoryData* m_oppositeTrajectoryData;
        bool m_fixedTarget = false;
    public:
        LIC<3> sampleExtendedSubTrajectory(float animationTime, float duration);
        EETrajectory getSubTrajectory(float animationTime);
        EETrajectory getSubTrajectoryByID(int subTrajectoryID);
        int getSubTrajectoryNum() { return m_originalSubTrajectories.size(); }
        LIC<3> const& getSavedPositions() { return m_savedPositions; }
        bool motionInitialized() { return m_motionInitialized; }
        float getSupportHeight(FrameIndex frame) { return m_supportHeights[frame]; }
        EETrajectory& getTargetTrajectory() { return m_targetTrajectory; }
        void setTargetTrajectory(LIC<3> curve, TrajectoryType trajectoryType, int subTrajectoryID) { 
            m_targetTrajectory = EETrajectory(curve, trajectoryType, subTrajectoryID); }
        void init(IKRigConfig* config, EEGlobalTrajectoryData* opposite);
        EEGlobalTrajectoryData* getOppositeTrajectoryData();
        bool isTargetFixed() { return m_fixedTarget; }
        void refresh();
    };


    struct TGData {
        LIC<3>* varCurve;
        LIC<3> baseCurve;
        std::vector<int> pointIndexes;
        std::vector<float> minValues; // tamano D*pointIndexes.size()
        float descentRate;
        float targetPosDelta;
        int maxIterations;
    };
    class StrideCorrector {
        GradientDescent<TGData> m_gradientDescent;
        TGData m_tgData;
    public:
        StrideCorrector() = default;
        void init();
        void correctStride(LIC<3>& targetCurve, LIC<3>& originalCurve, 
            EnvironmentData& environmentData,
            ComponentManager<TransformComponent>& transformManager,
            ComponentManager<StaticMeshComponent>& staticMeshManager);

    };
    
}





#endif