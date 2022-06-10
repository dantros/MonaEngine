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
    class ForwardKinematics;

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
        glm::vec3 getRotationAxis() const { return m_rotationAxis; }
    };
    class IKRigConfig {
        friend class IKRig;
    private:
        std::vector<std::vector<JointRotation>> m_baseJointRotations;
        std::vector<JointRotation> m_dynamicJointRotations;
        std::vector<glm::vec3> m_jointScales;
        std::vector<glm::vec3> m_jointPositions;
        std::vector<float> m_timeStamps;
        AnimationIndex m_animIndex = -1;
        ForwardKinematics* m_forwardKinematics;
        float m_currentTime = -1;
        int m_nextFrameIndex = -1;
    public:
        const std::vector<JointRotation>& getBaseJointRotations() const { return m_baseJointRotations[m_nextFrameIndex]; }
        const std::vector<JointRotation>& getDynamicJointRotations() const { return m_dynamicJointRotations; }
        const std::vector<glm::vec3>& getJointScales() const { return m_jointScales; }
        const std::vector<glm::vec3>& getJointPositions() const { return m_jointPositions; }
        const std::vector<float>& getTimeStamps() const { return m_timeStamps; }
        AnimationIndex getAnimIndex() const { return m_animIndex; }
        std::vector<JointRotation>* getDynamicJointRotationsPtr() { return &m_dynamicJointRotations;  }
        float getCurrentTime() const { return m_currentTime; }
        int getNextFrameIndex() const { return m_nextFrameIndex; }
        IKRigConfig(std::shared_ptr<AnimationClip> animation, AnimationIndex animIndex, ForwardKinematics* fk);
        glm::vec3 getModelSpacePosition(JointIndex jointIndex, bool useDynamicRotations);
        std::vector<glm::mat4> getModelSpaceTransforms(bool useDynamicRotations);
        std::vector<glm::mat4> getJointSpaceTransforms(bool useDynamicRotations);
    };

    struct JointData {
        float minAngle = -90;
        float maxAngle = 90;
        float weight = 1;
        bool enableIKRotation = false;
    };
    struct ChainEnds {
        std::string baseJointName;
        std::string endEffectorName;
    };
    struct IKChain {
        std::string name;
        std::vector<JointIndex> joints;
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
        friend class IKRig;
        float m_weight = 1;
        float m_minAngle = -90;
        float m_maxAngle = 90;
        std::string m_jointName;
        int m_jointIndex = -1;
        IKNode* m_parent = nullptr;
    public:
        IKNode() = default;
        IKNode(std::string jointName, JointIndex jointIndex, IKNode * parent = nullptr, float weight = 1);
        IKNode* getParent() const { return m_parent; }
        glm::vec2 getMotionRange() const { return glm::vec2(m_minAngle, m_maxAngle); }
        JointIndex getIndex() const { return m_jointIndex; }
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

}










#endif