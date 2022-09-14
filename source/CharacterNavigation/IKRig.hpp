#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"
#include "Kinematics.hpp"

#define MAX_EXPECTED_NUMBER_OF_ANIMATIONS_PER_IKRIG 20

namespace Mona{
    
    typedef int AnimationIndex;
    typedef int JointIndex;
    typedef int ChainIndex;
    class Skeleton;
    class IKRig{
        friend class IKRigController;
        friend class AnimationValidator;
        friend class IKNavigationComponent;
        friend class DebugDrawingSystem_ikNav;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle);
            IKAnimation* getIKAnimation(AnimationIndex animIndex) { return &m_ikAnimations[animIndex]; };
            const std::vector<int>& getTopology() const;
            const std::vector<std::string>& getJointNames() const;
            IKChain* getIKChain(ChainIndex chainIndex) { return &m_ikChains[chainIndex]; };
            int getChainNum() { return m_ikChains.size(); }
            JointIndex getHipJoint() { return m_hipJoint; }
            glm::vec3 getFrontVector() { return m_frontVector; }
            glm::vec3 getUpVector() { return m_upVector; }
            glm::vec3 getRightVector() { return m_rightVector; }
            float getRotationAngle() { return m_rotationAngle; }
            float getRigHeight() { return m_rigHeight; }
            float getRigScale() { return m_rigScale; }
            void setAngularSpeed(float angularSpeed) { m_angularSpeed = angularSpeed; }
            float getAngularSpeed() { return m_angularSpeed; }
            InnerComponentHandle getTransformHandle() { return m_transformHandle; }
            void init();
            void fixAnimation(IKAnimation* ikAnim, FrameIndex fixedFrame);
            void resetAnimation(IKAnimation* ikAnim);
            void resetAnimation(AnimationIndex animIndex);
        private:
            // Informacion de configuracion del IKRig por cada animacion
            std::vector<IKAnimation> m_ikAnimations;
            std::shared_ptr<Skeleton> m_skeleton;
            // Direccion frontal base de movimiento del rig
            glm::vec3 m_frontVector = glm::vec3(0.0f, 1.0f, 0.0f);
            glm::vec3 m_upVector = glm::vec3(0, 0, 1);
            glm::vec3 m_rightVector = glm::vec3(1.0f, 0.0f, 0.0f);
            // rapidez de giro
            float m_angularSpeed = 0;
            float m_rotationAngle = 0;
            TrajectoryGenerator m_trajectoryGenerator;
            InnerComponentHandle m_transformHandle;
            ForwardKinematics m_forwardKinematics;
            InverseKinematics m_inverseKinematics;
            // Altura aproximada del rig(model space), para calculos con distancias relativas
            float m_rigHeight;
            // Escala global del rig
            float m_rigScale;
            glm::vec3 m_initialPosition;
            // Arreglo de cadenas IK
            std::vector<IKChain> m_ikChains;
            // Indice de la cadera
            JointIndex m_hipJoint;
            IKChain buildIKChain(ChainEnds chainEnds, std::string chainName);
            std::vector<std::pair<JointIndex, float>> calculateRotationAngles(AnimationIndex animIndex);
            void calculateTrajectories(AnimationIndex animIndex,
                ComponentManager<TransformComponent>& transformManager,
                ComponentManager<StaticMeshComponent>& staticMeshManager);
            std::vector<JointIndex> getJointChildren(JointIndex jointIndex);
    };

}


#endif