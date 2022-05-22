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
    typedef Eigen::Quaternion<float> Quaternion;
    typedef Eigen::AngleAxis<float> AngleAxis;
    typedef Eigen::Matrix<float, 1, 2> Vector2f;
    typedef Eigen::Matrix<float, 3, 3> Matrix3f;

    struct JointRotation {
    private:
        AngleAxis m_angleAxis;
        Quaternion m_quatRotation;
    public:
        JointRotation();
        JointRotation(Vector3f rotationAxis, float rotationAngle);
        JointRotation(Quaternion quatRotation);
        void setRotation(Quaternion rotation);
        void setRotation(Vector3f rotationAxis, float rotationAngle);
        Quaternion getQuatRotation() { return m_quatRotation; }
        float getRotationAngle() { return m_angleAxis.angle(); }
        Vector3f getRotationAxis() { return m_angleAxis.axis(); }
    };

    struct JointData {
        float minAngle = -90;
        float maxAngle = 90;
        float weight = 1;
        bool enableData = false;
    };
    struct ChainEnds {
        std::string startJointName;
        std::string endEffectorName;
    };
    struct RigData {
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
        void setJointData(std::string jointName, float minAngle, float maxAngle, float weight = 1, bool enableData =true);
        void enableJointData(std::string jointName, bool enableData);
        JointData getJointData(std::string jointName);
        bool isValid();
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
        std::string m_jointName;
        int m_jointIndex = -1;
        IKNode* m_parent = nullptr;
        JointRotation m_jointRotation_dmic;
    };
    
    class IKRig{
        friend class IKNavigationComponent;
        friend class IKRigConfig;
        public:
            IKRig() = default;
            IKRig(std::shared_ptr<BVHData> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
                InnerComponentHandle skeletalMeshHandle);
            std::vector<Vector3f> bvhModelSpacePositions(int frame, bool useTargetAnim=false);
            std::vector<Vector3f> dynamicModelSpacePositions(bool useTargetAnim = false);
            Vector3f bvhCenterOfMass(int frame, bool useTargetAnim = false);
            Vector3f dynamicCenterOfMass(bool useTargetAnim = false);
            bool dynamicRotationsValid();
        private:
            Vector3f _centerOfMass(std::vector<Vector3f> modelSpacePositions);
            std::vector<std::shared_ptr<BVHData>> m_bvhAnims;
            std::shared_ptr<BVHData> m_currentAnim = nullptr;
            std::shared_ptr<BVHData> m_targetAnim = nullptr;
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
            void addAnimation(std::shared_ptr<AnimationClip> animationClip, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr);
            int removeAnimation(std::shared_ptr<AnimationClip> animationClip);
            Vector3f getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
            void setLinearVelocity(Vector3f velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
    };

}


#endif