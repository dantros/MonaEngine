#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"

namespace Mona{
    
    class IKRig{
        friend class IKNavigationComponent;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
                InnerComponentHandle skeletalMeshHandle, AnimationController* animController);
            IKRigConfig* getAnimConfig(float time, AnimIndex animIndex);
            IKRigConfig createDynamicConfig(int animIndex);
            std::vector<Vector3f> modelSpacePositions(const IKRigConfig& rigConfig);
            Vector3f getCenterOfMass(const IKRigConfig& rigConfig);
            bool isConfigValid(const IKRigConfig& rigConfig);
        private:
            FootContacts findFootContactFrames(std::shared_ptr<AnimationClip> anim);
            std::vector<std::shared_ptr<AnimationClip>> m_animations;
            std::vector<IKRigConfig> m_animConfigurations;
            std::shared_ptr<Skeleton> m_skeleton;
            AnimationController* m_animationController;
            std::vector<int>& GetTopology() { return m_skeleton->m_parentIndices; }
            std::vector<std::string>& GetJointNames() { return m_skeleton->m_jointNames; }

            AnimIndex m_currentAnim = -1;
            AnimIndex m_targetAnim = -1;
            InnerComponentHandle m_rigidBodyHandle;
            InnerComponentHandle m_skeletalMeshHandle;
            EnvironmentData m_environmentData;
            IKRigConfigValidator m_configValidator;
            std::vector<IKNode> m_nodes;
            std::pair<int, int> m_leftLeg = { -1,-1 };
            std::pair<int, int> m_rightLeg = { -1,-1 };
            std::pair<int, int> m_leftFoot = { -1,-1 };
            std::pair<int, int> m_rightFoot = { -1,-1 };
            void addAnimation(std::shared_ptr<AnimationClip> animationClip, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr);
            int removeAnimation(std::shared_ptr<AnimationClip> animationClip);
            Vector3f getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            void setLinearVelocity(Vector3f velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
    };

}


#endif