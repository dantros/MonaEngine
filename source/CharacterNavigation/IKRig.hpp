#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include "IKRigBase.hpp"

namespace Mona{
    
    class IKRig{
        friend class IKNavigationComponent;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<BVHData> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
                InnerComponentHandle skeletalMeshHandle);
            IKRigConfig getBVHConfig(int frame, BVHIndex animIndex);
            IKRigConfig createDynamicConfig();
            std::vector<Vector3f> modelSpacePositions(IKRigConfig rigConfig);
            Vector3f getCenterOfMass(IKRigConfig rigConfig);
            bool isConfigValid(IKRigConfig rigConfig);
        private:
            FootContacts findFootContactFrames(std::shared_ptr<BVHData> anim);
            std::vector<std::shared_ptr<BVHData>> m_bvhAnims;
            std::vector<int> m_topology;
            std::vector<std::string> m_jointNames;
            std::vector<Vector3f> m_offsets;
            BVHIndex m_currentAnim = -1;
            BVHIndex m_targetAnim = -1;
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