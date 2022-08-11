#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"
#include "Kinematics.hpp"

namespace Mona{
    
    typedef int AnimationIndex;
    typedef int JointIndex;
    typedef int ChainIndex;
    class Skeleton;
    class IKRig{
        friend class IKRigController;
        friend class IKNavigationComponent;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle);
            IKRigConfig* getAnimationConfig(AnimationIndex animIndex) { return &m_animationConfigs[animIndex]; };
            const std::vector<int>& getTopology() const;
            const std::vector<std::string>& getJointNames() const;
            IKNode* getIKNode(JointIndex jointIndex) { return &m_nodes[jointIndex]; };
            IKChain* getIKChain(ChainIndex chainIndex) { return &m_ikChains[chainIndex]; };
            JointIndex getHipJoint() { return m_hipJoint; }
            glm::vec2 getFrontVector() { return m_frontVector; }
            glm::vec3 getUpVector() { return m_upVector; }
            float getRotationAngle() { return m_rotationAngle; }
            float getRigHeight() { return m_rigHeight; }
            void setAngularSpeed(float angularSpeed) { m_angularSpeed = angularSpeed; }
            float getAngularSpeed() { return m_angularSpeed; }
            InnerComponentHandle getTransformHandle() { return m_transformHandle; }
            void init(float rigScale);
        private:
            // Informacion de configuracion del IKRig por cada animacion
            std::vector<IKRigConfig> m_animationConfigs;
            std::shared_ptr<Skeleton> m_skeleton;

            AnimationIndex m_currentAnim = -1;
            AnimationIndex m_targetAnim = -1;
            // Direccion frontal base de movimiento del rig
            glm::vec2 m_frontVector = glm::vec2(0.0f, 1.0f);
            glm::vec3 m_upVector = glm::vec3(0, 0, 1);
            //glm::vec2 m_rightVector; (1,0)
            // rapidez de giro
            float m_angularSpeed = 0;
            float m_rotationAngle = 0;
            TrajectoryGenerator m_trajectoryGenerator;
            InnerComponentHandle m_transformHandle;
            ForwardKinematics m_forwardKinematics;
            InverseKinematics m_inverseKinematics;
            // Altura aproximada global del rig, para calculos con distancias relativas
            float m_rigHeight;
            // Arreglo de nodos (hay uno por cada articulacion)
            std::vector<IKNode> m_nodes;
            // Arreglo de cadenas IK
            std::vector<IKChain> m_ikChains;
            // Indice de la cadera
            JointIndex m_hipJoint;
            IKChain buildIKChain(ChainEnds chainEnds, std::string chainName);
            std::vector<std::pair<JointIndex, glm::fquat>> calculateRotations(AnimationIndex animIndex);
            void calculateTrajectories(AnimationIndex animIndex,
                ComponentManager<TransformComponent>& transformManager,
                ComponentManager<StaticMeshComponent>& staticMeshManager);
    };

}


#endif