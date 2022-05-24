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
                InnerComponentHandle skeletalMeshHandle);
            IKRigConfig getBVHConfig(int frame, AnimIndex animIndex);
            IKRigConfig createDynamicConfig();
            std::vector<Vector3f> modelSpacePositions(IKRigConfig rigConfig);
            Vector3f getCenterOfMass(IKRigConfig rigConfig);
            bool isConfigValid(IKRigConfig rigConfig);
        private:
            FootContacts findFootContactFrames(std::shared_ptr<AnimationClip> anim);
            std::vector<std::shared_ptr<AnimationClip>> m_animations;
            std::shared_ptr<Skeleton> m_skeleton;
            std::vector<int>& GetTopology() { return m_skeleton->m_parentIndices; }
            std::vector<std::string>& GetJointNames() { return m_skeleton->m_jointNames; }
            std::vector<Vector3f> m_offsets;
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