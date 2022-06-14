#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"
#include "Kinematics.hpp"

namespace Mona{
    
    typedef int AnimationIndex;
    typedef int JointIndex;
    class IKRig{
        friend class IKNavigationComponent;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
                InnerComponentHandle skeletalMeshHandle, AnimationController* animController, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr);
            void setIKRigConfigTime(float time, AnimationIndex animIndex);
            IKRigConfig* getAnimationConfig(AnimationIndex animIndex) { return &m_animationConfigs[animIndex]; };
            std::shared_ptr<AnimationClip> getAnimation(AnimationIndex animIndex) { return m_animations[animIndex]; };
            const std::vector<int>& getTopology() const { return m_skeleton->m_parentIndices; };
            const std::vector<std::string>& getJointNames() const { return m_skeleton->m_jointNames; };
            IKNode* getIKNode(JointIndex jointIndex) { return &m_nodes[jointIndex]; };

        private:
            std::vector<std::shared_ptr<AnimationClip>> m_animations;
            std::vector<IKRigConfig> m_animationConfigs;
            std::shared_ptr<Skeleton> m_skeleton;
            AnimationController* m_animationController;

            AnimationIndex m_currentAnim = -1;
            AnimationIndex m_targetAnim = -1;
            glm::vec3 m_linearVelocity;
            glm::vec3 m_angularVelocity;
            InnerComponentHandle m_rigidBodyHandle;
            InnerComponentHandle m_skeletalMeshHandle;
            EnvironmentData m_environmentData;
            ForwardKinematics m_forwardKinematics;
            InverseKinematics m_inverseKinematics;
            std::vector<IKNode> m_nodes;
            std::vector<IKChain> m_ikChains;
            void addAnimation(std::shared_ptr<AnimationClip> animationClip, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr);
            int removeAnimation(std::shared_ptr<AnimationClip> animationClip);
            glm::vec3 getRBLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            void setRBLinearVelocity(glm::vec3 velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            glm::vec3 getRBAngularVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            void setRBAngularVelocity(glm::vec3 velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            IKChain buildIKChain(ChainEnds chainEnds, std::string chainName);
            IKChain buildHipIKChain(std::string hipJointName);
            std::vector<IKChain*> getIKChainPtrs(bool includeHipChain=false);
            void UpdateEETrajectories(float timeStep, glm::vec3 rawLinVel, glm::vec3 rawAngVel);
    };

}


#endif