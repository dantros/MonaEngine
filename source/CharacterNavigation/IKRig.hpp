#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include <memory>
#include "BVHManager.hpp"
#include "EnvironmentData.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"

namespace Mona{
    typedef Eigen::Matrix<float, 1, 3> Vector3f;
    typedef Eigen::Matrix<float, 1, 2> Vector2f;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;

    struct JointData {
        float minAngle = -90;
        float maxAngle = 90;
        bool freeAxis = false;
        float weight = 1;
        bool enableData = false;
    };
    struct ChainEnds {
        std::string startJointName;
        std::string endEffectorName;
    };
    class RigData {
        friend class IKRig;
    private:
        std::unordered_map<std::string, JointData> jointData;
    public:
        ChainEnds spine;
        ChainEnds leftLeg;
        ChainEnds rightLeg;
        ChainEnds leftArm;
        ChainEnds rightArm;
        ChainEnds leftFoot;
        ChainEnds rightFoot;
        void setJointData(std::string jointName, float minAngle, float maxAngle, bool freeAxis = false, float weight = 1, bool enableData =true);
        void enableJointData(std::string jointName, bool enableData);
        JointData getJointData(std::string jointName);
    };

    class IKNode {
    public:
        IKNode() = default;
        IKNode(std::string jointName, int jointIndex, IKNode * parent = nullptr, float weight = 1);
    private:
        friend class IKRig;
        float m_weight = 1;
        float m_minAngle = -90;
        float m_maxAngle = 90;
        bool m_freeAxis = false;
        std::string m_jointName;
        int m_jointIndex = -1;
        IKNode* m_parent = nullptr;
        Vector3f m_rotationAxis;
    };

    class IKRig{
        friend class IKNavigationComponent;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<BVHData> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle, 
                InnerComponentHandle skeletalMeshHandle, bool adjustFeet);
        private:
            std::vector<std::shared_ptr<BVHData>> m_bvhAnims;
            int m_currentClipIndex = -1;
            int m_targetClipIndex = -1;
            bool m_adjustFeet;
            InnerComponentHandle m_rigidBodyHandle;
            InnerComponentHandle m_skeletalMeshHandle;
            EnvironmentData m_environmentData;
            std::vector<IKNode> m_nodes;
            std::pair<int, int> m_spine = { -1,-1 };
            std::pair<int, int> m_leftLeg = { -1,-1 };
            std::pair<int, int> m_rightLeg = { -1,-1 };
            std::pair<int, int> m_leftArm = { -1,-1 };
            std::pair<int, int> m_rightArm = { -1,-1 };
            std::pair<int, int> m_leftFoot = { -1,-1 };
            std::pair<int, int> m_rightFoot = { -1,-1 };
            void addAnimation(std::shared_ptr<BVHData> animation);
            int removeAnimation(std::shared_ptr<BVHData> animation);
            Vector3f getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
    };

}


#endif