#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"

namespace Mona{
    
    class IKRig{
        friend class IKNavigationComponent;
        friend class InverseKinematics;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
                InnerComponentHandle skeletalMeshHandle, AnimationController* animController);
            void setIKRigConfigTime(float time, AnimationIndex animIndex);
            bool isConfigValid(const IKRigConfig& rigConfig);
            const IKRigConfig& GetAnimationConfig(AnimationIndex animIndex) const { return m_animationConfigs[animIndex]; }
            std::shared_ptr<AnimationClip> GetAnimation(AnimationIndex animIndex) { return m_animations[animIndex]; }
            const std::vector<int>& GetTopology() const { return m_skeleton->m_parentIndices; }
            const std::vector<std::string>& GetJointNames() const { return m_skeleton->m_jointNames; }
        private:
            FootContacts findFootContactFrames(std::shared_ptr<AnimationClip> anim);
            std::vector<std::shared_ptr<AnimationClip>> m_animations;
            std::vector<IKRigConfig> m_animationConfigs;
            std::shared_ptr<Skeleton> m_skeleton;
            AnimationController* m_animationController;

            AnimationIndex m_currentAnim = -1;
            AnimationIndex m_targetAnim = -1;
            InnerComponentHandle m_rigidBodyHandle;
            InnerComponentHandle m_skeletalMeshHandle;
            EnvironmentData m_environmentData;
            IKRigConfigValidator m_configValidator;
            std::vector<IKNode> m_nodes;
            ChainEnds m_leftLeg;
            ChainEnds m_rightLeg;
            ChainEnds m_leftFoot;
            ChainEnds m_rightFoot;
            void addAnimation(std::shared_ptr<AnimationClip> animationClip, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr);
            int removeAnimation(std::shared_ptr<AnimationClip> animationClip);
            glm::vec3 getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            void setLinearVelocity(glm::vec3 velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
    };

}


#endif