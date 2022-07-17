#pragma once
#ifndef IKRIGBASE_HPP
#define IKRIGBASE_HPP

#include <memory>
#include "EnvironmentData.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"
#include "ParametricCurves.hpp"

namespace Mona {
    typedef int AnimationIndex;
    typedef int JointIndex;
    typedef int FrameIndex;
    typedef int ChainIndex;
    class ForwardKinematics;

    class JointRotation {
    private:
        // Rotacion en formato cuaternion
        glm::fquat m_quatRotation;
        // Eje de rotacion
        glm::vec3 m_rotationAxis;
        // Angulo de rotacion
        float m_rotationAngle;
    public:
        JointRotation();
        JointRotation(float rotationAngle, glm::vec3 rotationAxis);
        JointRotation(glm::fquat quatRotation);
        void setRotation(glm::fquat rotation);
        void setRotation(float rotationAngle, glm::vec3 rotationAxis);
        void setRotationAngle(float rotationAngle);
        void setRotationAxis(glm::vec3 rotationAxis);
        glm::fquat getQuatRotation() const { return m_quatRotation; }
        float getRotationAngle() const { return m_rotationAngle; }
        glm::vec3 getRotationAxis() const { return m_rotationAxis; }
    };
    struct HipGlobalTrajectoryData {
        friend class IKRigController;
        LIC<3> m_originalTrajectory;
        // Angulos de rotacion originales
        LIC<1> m_originalRotationAngles;
        // Ejes de rotacion originales
        LIC<3> m_originalRotationAxes;
        // Traslaciones originales
        LIC<3> m_originalTranslations;
        // Direccion original de la cadera
        glm::vec2 m_originalFrontVector;
        // Trayectoria objetivo
        LIC<3> m_targetTrajectory;
        std::vector<glm::vec3> m_savedPositions;
    public:
        LIC<3> getOriginalTrajectory() { return m_originalTrajectory; }
        LIC<1> getOriginalRotationAngles() { return m_originalRotationAngles; }
        LIC<3> getOriginalRotationAxes() { return m_originalRotationAxes; }
        LIC<3> getOriginalTranslations() { return m_originalTranslations; }
        glm::vec2 getOriginalFrontVector() { return m_originalFrontVector; }
        glm::vec3 getSavedPosition(FrameIndex frame) { return m_savedPositions[frame]; }
        LIC<3> getTargetTrajectory() { return m_targetTrajectory; }
        void setTargetTrajectory(LIC<3> targetTrajectory) { m_targetTrajectory = targetTrajectory; };
    };

    enum class TrajectoryType {
        STATIC,
        DYNAMIC
    };
    enum class  AnimationType {
        IDLE,
        MOVING
    };


    class EETrajectory {
        friend class IKRigController;
        friend class EETrajectoryData;
        LIC<3> m_trajectory;
        TrajectoryType m_trajectoryType;
        // Tiempo en que ocurre la maxima altura de la cadera (dentro del rango t de esta trayectoria)
        float m_hipMaxAltitudeTimeFraction = -1;
    public:
        EETrajectory() = default;
        EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType);
        bool isDynamic() { return m_trajectoryType == TrajectoryType::DYNAMIC; }
        LIC<3>& getEETrajectory() { return m_trajectory; }
        float getHipMaxAltitudeTimeFraction() { return m_hipMaxAltitudeTimeFraction; }
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
    class IKRigConfig {
        friend class IKRig;
        friend class IKRigController;
    private:
        // Clip de animacion asociado a esta configuracion
        std::shared_ptr<AnimationClip> m_animationClip;
        // Rotaciones para cada joint de la animacion base para cada frame 
        std::vector<std::vector<JointRotation>> m_baseJointRotations;
        // Rotaciones modificables para cada joint
        std::vector<std::vector<JointRotation>> m_dynamicJointRotations;
        // Escalas para cada joint de la animacion base (fijas)
        std::vector<glm::vec3> m_jointScales;
        // Posiciones para cada joint de la animacion base (fijas)
        std::vector<glm::vec3> m_jointPositions;
        // Timestamps para las rotaciones indexadas por frame
        std::vector<float> m_timeStamps;
        // Indice de la animacion que le corresponde a esta configuracion
        AnimationIndex m_animIndex = -1;
        ForwardKinematics* m_forwardKinematics;
        // Data de trayectoria para cada ikChain (mantiene orden del arreglo original de cadenas)
        std::vector<EEGlobalTrajectoryData> m_ikChainTrajectoryData;
        // Data de trayectoria para la cadera
        HipGlobalTrajectoryData m_hipTrajectoryData;
        // Tiempo actual de la animacion
        float m_currentReproductionTime = -1;
        // Indica el frame mas reciente de la animacion
        FrameIndex m_currentFrameIndex = -1;
        // Indice del siguiente frame de la animacion
        FrameIndex m_nextFrameIndex = -1;
        // Indica si es necesario actualizar las rotaciones de las joints
        bool m_requiresIKUpdate = true;
        // Numero de veces que la animacion se ha reproducido
        int m_reproductionCount = 0;
        //
        AnimationType m_animationType;
        // ajustar animationTime input al rango correspondiente (del arreglo de timeStamps)
        float adjustAnimationTime(float animationTime);
    public:
        IKRigConfig(std::shared_ptr<AnimationClip> animation, AnimationIndex animIndex, ForwardKinematics* fk);
        const std::vector<JointRotation>& getBaseJointRotations() const { return m_baseJointRotations[m_nextFrameIndex]; }
        const std::vector<JointRotation>& getBaseJointRotations(FrameIndex frame) const { return m_baseJointRotations[frame]; }
        const std::vector<JointRotation>& getDynamicJointRotations() const { return m_dynamicJointRotations[m_nextFrameIndex]; }
        const std::vector<JointRotation>& getDynamicJointRotations(FrameIndex frame) const { return m_dynamicJointRotations[frame]; }
        const std::vector<glm::vec3>& getJointScales() const { return m_jointScales; }
        const std::vector<glm::vec3>& getJointPositions() const { return m_jointPositions; }
        float getReproductionTime(float animationTime, int repCountOffset = 0);
        float getReproductionTime(FrameIndex frame, int repCountOffset = 0);
        float getAnimationTime(FrameIndex frame);
        float getAnimationTime(float reproductionTime);
        FrameIndex getFrame(float animationTime);
        float getAnimationDuration() { return m_animationClip->GetDuration(); }
        int getFrameNum() { return m_timeStamps.size(); }
        int getReproductionCount() const { return m_reproductionCount; }
        std::vector<JointRotation>* getDynamicJointRotationsPtr() { return &(m_dynamicJointRotations[m_nextFrameIndex]);  }
        float getCurrentReproductionTime() const { return m_currentReproductionTime; }
        FrameIndex getNextFrameIndex() const { return m_nextFrameIndex; }
        FrameIndex getCurrentFrameIndex() const { return m_currentFrameIndex; }
        std::vector<glm::mat4> getModelSpaceTransforms(bool useDynamicRotations);
        glm::mat4 getModelSpaceTransform(JointIndex jointIndex, FrameIndex frame, bool useDynamicRotations);
        std::vector<glm::vec3> getModelSpacePositions(bool useDynamicRotations);
        std::vector<glm::vec3> getModelSpacePositions(FrameIndex frame, bool useDynamicRotations);
        std::vector<glm::vec3> getCustomSpacePositions(glm::mat4 baseTransform, bool useDynamicRotations);
        std::vector<glm::mat4> getJointSpaceTransforms(bool useDynamicRotations);
        EEGlobalTrajectoryData* getEETrajectoryData(ChainIndex chainIndex) { return &(m_ikChainTrajectoryData[chainIndex]); }
        HipGlobalTrajectoryData* getHipTrajectoryData() { return &m_hipTrajectoryData; }
        AnimationType getAnimationType() { return m_animationType; }
    };

    struct JointData {
        // Minimo angulo de rotacion de la articulacion (grados)
        float minAngle = -90;
        // Maximo angulo de rotacion de la articulacion (grados)
        float maxAngle = 90;
        float weight = 1;
        bool enableIKRotation = false;
    };
    struct ChainEnds {
        // Nombre de la articulacion base de la cadena (al ser la base no se le aplica IK)
        std::string baseJointName;
        // Nombre de la articulacion final de la cadena
        std::string endEffectorName;
    };
    class IKChain {
        friend class IKRig;
        // Nombre de la cadena
        std::string m_name;
        // Articulaciones que conforman la cadena, desde el origen hasta el ee
        std::vector<JointIndex> m_joints;
        // Objetivo actual para el end effector (donde debe posicionarse) (model space)
        glm::vec3 m_currentEETarget;
    public:
        IKChain() = default;
        const std::string& getName() const { return m_name; };
        const std::vector<JointIndex>& getJoints() const { return m_joints; };
        const glm::vec3& getCurrentEETarget() const { return m_currentEETarget; };
    };
    struct RigData {
        friend class IKRig;
    private:
        std::unordered_map<std::string, JointData> jointData;
    public:
        ChainEnds leftLeg;
        ChainEnds rightLeg;
        ChainEnds leftFoot;
        ChainEnds rightFoot;
        float scale;
        std::string hipJointName;
        void setJointData(std::string jointName, float minAngle, float maxAngle, float weight = 1, bool enableData = true);
        void enableJointData(std::string jointName, bool enableData);
        JointData getJointData(std::string jointName);
        bool isValid();
    };

    class IKNode {
        friend class IKRig;
        float m_weight = 1;
        // Minimo angulo de rotacion de la articulacion (radianes)
        float m_minAngle = -90;
        // Maximo angulo de rotacion de la articulacion (radianes)
        float m_maxAngle = 90;
        // Nombre de la articulacion
        std::string m_jointName;
        // Indice de la articulacion
        int m_jointIndex = -1;
        // Puntero al nodo padre
        IKNode* m_parent = nullptr;
    public:
        IKNode() = default;
        IKNode(std::string jointName, JointIndex jointIndex, IKNode * parent = nullptr, float weight = 1);
        IKNode* getParent() const { return m_parent; }
        glm::vec2 getMotionRange() const { return glm::vec2(m_minAngle, m_maxAngle); }
        JointIndex getIndex() const { return m_jointIndex; }
    };

}










#endif