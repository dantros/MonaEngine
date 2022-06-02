#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"

namespace Mona{
    
    class IKRig{
        friend class IKNavigationComponent;
        friend class ForwardKinematics;
        friend class GradientDescentIK;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
                InnerComponentHandle skeletalMeshHandle, AnimationController* animController);
            void setIKRigConfigTime(float time, AnimationIndex animIndex);
            std::vector<Vector3f> modelSpacePositions(const IKRigConfig& rigConfig);
            bool isConfigValid(const IKRigConfig& rigConfig);
        private:
            FootContacts findFootContactFrames(std::shared_ptr<AnimationClip> anim);
            std::vector<std::shared_ptr<AnimationClip>> m_animations;
            std::vector<IKRigConfig> m_animationConfigs;
            std::shared_ptr<Skeleton> m_skeleton;
            AnimationController* m_animationController;
            const std::vector<int>& GetTopology() const { return m_skeleton->m_parentIndices; }
            const std::vector<std::string>& GetJointNames() const { return m_skeleton->m_jointNames; }

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
            Vector3f getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            void setLinearVelocity(Vector3f velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
    };

}


#endif