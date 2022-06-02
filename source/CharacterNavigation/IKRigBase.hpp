#pragma once
#ifndef IKRIGBASE_HPP
#define IKRIGBASE_HPP

#include <memory>
#include "EnvironmentData.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"

namespace Mona {
    typedef int AnimationIndex;
    typedef int JointIndex;

    struct JointRotation {
    private:
        glm::fquat m_quatRotation;
        glm::vec3 m_rotationAxis;
        float m_rotationAngle;
    public:
        JointRotation();
        JointRotation(float rotationAngle, glm::vec3 rotationAxis);
        JointRotation(glm::fquat quatRotation);
        void setRotation(glm::fquat rotation);
        void setRotation(float rotationAngle, glm::vec3 rotationAxis);
        void setRotationAngle(float rotationAngle);
        void setRotationAxis(glm::vec3 rotationAxis);
        glm::fquat getQuatRotation() const { return m_quatRotation; }
        float getRotationAngle() const { return m_rotationAngle; }
        Vector3f getRotationAxis() const { return m_rotationAxis; }
    };
    class IKRigConfig {
        friend class IKRig;
    private:
        std::vector<JointRotation> m_baseJointRotations;
        std::vector<JointRotation> m_dynamicJointRotations;
        std::vector<glm::vec3> m_jointScales;
        std::vector<glm::vec3> m_jointPositions;
        std::vector<float> m_timeStamps;
        AnimationIndex m_animIndex = -1;
        float m_currentTime = -1;
    public:
        const std::vector<JointRotation>& getBaseJointRotations() const { return m_baseJointRotations; }
        const std::vector<glm::vec3>& getJointScales() const { return m_jointScales; }
        const std::vector<glm::vec3>& getJointPositions() const { return m_jointPositions; }
        const std::vector<float>& getTimeStamps() const { return m_timeStamps; }
        std::vector<JointRotation>* getDynamicJointRotationsPtr() { return &m_dynamicJointRotations;  }
        float getCurrentTime() const { return m_currentTime; }
        IKRigConfig(std::shared_ptr<AnimationClip> animation, AnimationIndex animIndex);
    };

    struct JointData {
        float minAngle = -90;
        float maxAngle = 90;
        float weight = 1;
        bool enableIKRotation = false;
    };
    struct ChainEnds {
        friend class IKRig;
        std::string startJointName;
        std::string endEffectorName;
    private:
        JointIndex startJointIndex = -1;
        JointIndex endEffectorIndex = -1;
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
        IKNode(std::string jointName, JointIndex jointIndex, IKNode* parent = nullptr, float weight = 1);
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