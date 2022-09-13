#pragma once
#ifndef IKRIGBASE_HPP
#define IKRIGBASE_HPP

#include <memory>
#include "TrajectoryGenerator.hpp"

namespace Mona {
    typedef int AnimationIndex;
    typedef int JointIndex;
    typedef int FrameIndex;
    typedef int ChainIndex;
    class ForwardKinematics;
    class AnimationClip;    

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
    enum class  AnimationType {
        IDLE,
        WALKING
    };
    class IKAnimation {
        friend class IKRig;
        friend class IKRigController;
        friend class AnimationValidator;
        friend class DebugDrawingSystem_ikNav;
        friend class TrajectoryGenerator;
    private:
        // Indica si la animacion asociada esta activa
        bool m_active = false;
        // Clip de animacion asociado a esta configuracion
        std::shared_ptr<AnimationClip> m_animationClip;
        // Indices de las articulaciones presentes en la animacion. Ordenados de acuerdo a la toplogia.
        std::vector<JointIndex> m_jointIndices;
        // Rotaciones para cada joint de la animacion base para cada frame 
        std::vector<std::vector<JointRotation>> m_baseJointRotations;
        // Rotacion modificable por cada joint
        std::vector<JointRotation> m_variableJointRotations;
        // Historial de angulos variables para cada joint
        std::vector<LIC<1>> m_savedAngles;
        // Escalas para cada joint de la animacion base (fijas)
        std::vector<glm::vec3> m_jointScales;
        // Posiciones para cada joint de la animacion base (fijas)
        std::vector<glm::vec3> m_jointPositions;
        // Indice de la animacion que le corresponde a esta configuracion
        AnimationIndex m_animIndex = -1;
        ForwardKinematics* m_forwardKinematics;
        // Data de trayectoria para cada ikChain (mantiene orden del arreglo original de cadenas)
        std::vector<EEGlobalTrajectoryData> m_eeTrajectoryData;
        // Data de trayectoria para la cadera
        HipGlobalTrajectoryData m_hipTrajectoryData;
        // Tiempo actual de la animacion
        float m_currentReproductionTime = 0;
        // Indica el frame mas reciente de la animacion
        FrameIndex m_currentFrameIndex = -1;
        // Indice del siguiente frame de la animacion
        FrameIndex m_nextFrameIndex = -1;
        // Indica si es necesario actualizar las rotaciones de las joints
        bool m_onNewFrame = true;
        // Numero de veces que la animacion se ha reproducido
        int m_reproductionCount = 0;
        // Tipo de la animacion
        AnimationType m_animationType;
        FrameIndex m_fixedMovementFrame = -1;
        void refreshSavedAngles(JointIndex jointIndex);
    public:
        IKAnimation(std::shared_ptr<AnimationClip> animationClip, AnimationType animationType, 
            AnimationIndex animIndex, ForwardKinematics* fk);
        const std::vector<JointRotation>& getBaseJointRotations(FrameIndex frame) const { return m_baseJointRotations[frame]; }
        std::vector<JointRotation>* getVariableJointRotations() { return &m_variableJointRotations; }
        const std::vector<glm::vec3>& getJointScales() const { return m_jointScales; }
        const std::vector<glm::vec3>& getJointPositions() const { return m_jointPositions; }
        const std::vector<float>& getTimeStamps();
        float getReproductionTime(float extendedAnimationTime, int repCountOffset = 0);
        float getReproductionTime(FrameIndex frame, int repCountOffset = 0);
        float getAnimationTime(FrameIndex frame);
        float getAnimationTime(float reproductionTime);
        FrameIndex getFrame(float extendedAnimationTime);
        float getAnimationDuration();
        int getFrameNum() { return getTimeStamps().size(); }
        int getReproductionCount() const { return m_reproductionCount; }
        float getCurrentReproductionTime() const { return m_currentReproductionTime; }
        FrameIndex getNextFrameIndex() const { return m_nextFrameIndex; }
        FrameIndex getCurrentFrameIndex() const { return m_currentFrameIndex; }
        std::vector<glm::mat4> getEEListModelSpaceVariableTransforms(std::vector<JointIndex> eeList, std::vector<glm::mat4>* outJointSpaceTransforms = nullptr);
        std::vector<glm::mat4> getEEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, float reproductionTime, std::vector<glm::mat4>* outJointSpaceTransforms = nullptr);
        EEGlobalTrajectoryData* getEETrajectoryData(ChainIndex chainIndex) { return &(m_eeTrajectoryData[chainIndex]); }
        HipGlobalTrajectoryData* getHipTrajectoryData() { return &m_hipTrajectoryData; }
        AnimationType getAnimationType() { return m_animationType; }
        // ajustar animationTime input al rango correspondiente (del arreglo de timeStamps)
        float adjustAnimationTime(float extendedAnimationTime);
        std::vector<JointIndex>const& getJointIndices() { return m_jointIndices; }
        bool hasJoint(JointIndex joint);
        bool isActive() { return m_active; }
        bool isMovementFixed();
        FrameIndex getFixedMovementFrame() { return m_fixedMovementFrame; }
        LIC<1>const& getSavedAngles(JointIndex jointIndex) { return m_savedAngles[jointIndex]; }
        void setVariableJointRotations(FrameIndex frame);
        void refresh();
    };
    struct ChainEnds {
        // Nombre de la articulacion base de la cadena
        std::string baseJointName;
        // Nombre de la articulacion final de la cadena
        std::string endEffectorName;
    };
    class IKChain {
        friend class IKRig;
        // Nombre de la cadena
        std::string m_name;
        // Articulaciones que conforman la cadena, desde su origen hasta el ee
        std::vector<JointIndex> m_joints;
        // Articulacion padre de la cadena (no es parte de la cadena)
        JointIndex m_parentJoint;
        // Objetivo actual para el end effector (donde debe posicionarse) (model space)
        glm::vec3 m_currentEETarget;
        // Cadena opuesta a la actual (ej: pierna izquierda a pierna derecha). Para ajustar posiciones relativas.
        ChainIndex m_opposite;
    public:
        IKChain() = default;
        const std::string& getName() const { return m_name; };
        const std::vector<JointIndex>& getJoints() const { return m_joints; };
        JointIndex getParentJoint() { return m_parentJoint; }
        JointIndex getEndEffector() { return m_joints.back(); }
        const glm::vec3& getCurrentEETarget() const { return m_currentEETarget; };
        void setCurrentEETarget(glm::vec3 currentEETarget) { m_currentEETarget = currentEETarget; }
        ChainIndex getOpposite() { return m_opposite; }
    };

    struct RigData {
        friend class IKRig;
    public:
        ChainEnds leftLeg;
        ChainEnds rightLeg;
        std::string hipJointName;
        float initialRotationAngle = 0.0f;
        glm::vec3 initialPosition = glm::vec3(0);
        float scale = 1;
    };

}










#endif