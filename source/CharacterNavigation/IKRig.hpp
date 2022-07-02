#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"
#include "Kinematics.hpp"
#include "TrajectoryGenerator.hpp"

namespace Mona{
    
    typedef int AnimationIndex;
    typedef int JointIndex;
    typedef int ChainIndex;
    class IKRig{
        friend class IKRigController;
        friend class IKNavigationComponent;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle skeletalMeshHandle);
            IKRigConfig* getAnimationConfig(AnimationIndex animIndex) { return &m_animationConfigs[animIndex]; };
            const std::vector<int>& getTopology() const { return m_skeleton->m_parentIndices; };
            const std::vector<std::string>& getJointNames() const { return m_skeleton->m_jointNames; };
            IKNode* getIKNode(JointIndex jointIndex) { return &m_nodes[jointIndex]; };
            IKChain* getIKChain(ChainIndex chainIndex) { return &m_ikChains[chainIndex]; };

        private:
            // Informacion de configuracion del IKRig por cada animacion
            std::vector<IKRigConfig> m_animationConfigs;
            std::shared_ptr<Skeleton> m_skeleton;

            AnimationIndex m_currentAnim = 0;
            AnimationIndex m_targetAnim = -1;
            glm::vec3 m_linearVelocity;
            glm::vec3 m_angularVelocity;
            InnerComponentHandle m_skeletalMeshHandle;
            TrajectoryGenerator m_trajectoryGenerator;
            ForwardKinematics m_forwardKinematics;
            InverseKinematics m_inverseKinematics;
            // Altura aproximada del rig, para calculos con distancias relativas
            float m_rigHeight;
            // Arreglo de nodos (hay uno por cada articulacion)
            std::vector<IKNode> m_nodes;
            // Arreglo de cadenas IK
            std::vector<IKChain> m_ikChains;
            // Indice de la cadera
            JointIndex m_hipJoint;
            IKChain buildIKChain(ChainEnds chainEnds, std::string chainName);
            IKChain buildHipIKChain(std::string hipJointName);
            std::vector<std::pair<JointIndex, glm::fquat>> calculateRotations(AnimationIndex animIndex);
            void calculateEETrajectories(AnimationIndex animIndex, std::vector<ChainIndex> ikChains);
    };

}


#endif