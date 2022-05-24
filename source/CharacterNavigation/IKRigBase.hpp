#pragma once
#ifndef IKRIGBASE_HPP
#define IKRIGBASE_HPP

#include <memory>
#include "BVHManager.hpp"
#include "EnvironmentData.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"

namespace Mona {
    typedef Eigen::Matrix<float, 1, 3> Vector3f;
    typedef Eigen::Quaternion<float> Quaternion;
    typedef Eigen::AngleAxis<float> AngleAxis;
    typedef Eigen::Matrix<float, 1, 2> Vector2f;
    typedef int AnimIndex;

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
    typedef std::vector<JointRotation> IKRigConfig;

    struct JointData {
        float minAngle = -90;
        float maxAngle = 90;
        float weight = 1;
        bool enableIKRotation = false;
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
        ChainEnds leftLeg;
        ChainEnds rightLeg;
        ChainEnds leftFoot;
        ChainEnds rightFoot;
        void setJointData(std::string jointName, float minAngle, float maxAngle, float weight = 1, bool enableData = true);
        void enableJointData(std::string jointName, bool enableData);
        JointData getJointData(std::string jointName);
        bool isValid();
    };

    class IKNode {
    public:
        IKNode() = default;
        IKNode(std::string jointName, int jointIndex, IKNode* parent = nullptr, float weight = 1);
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

    struct FootContacts {
        std::vector<int> leftLegUp;
        std::vector<int> leftLegDown;
        std::vector<int> leftFootUp;
        std::vector<int> leftFootDown;
        std::vector<int> rightLegUp;
        std::vector<int> rightLegDown;
        std::vector<int> rightFootUp;
        std::vector<int> rightFootDown;
    };

    class IKRigConfigValidator {
        friend class IKRig;
        std::vector<IKNode>* m_nodes;
        std::vector<int>* m_topology;
        IKRigConfigValidator() = default;
        IKRigConfigValidator(std::vector<IKNode>* nodesPtr, std::vector<int>* topologyPtr);
        bool isConfigValid(IKRigConfig rigConfig);
    };

}










#endif