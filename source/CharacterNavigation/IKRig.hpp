#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include <memory>
#include "BVHManager.hpp"
#include "EnvironmentData.hpp"

namespace Mona{
    typedef Eigen::Matrix<float, 1, 3> Vector3f;
    typedef Eigen::Matrix<float, 1, 2> Vector2f;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;

    struct JointData {
        float minAngle = -90;
        float maxAngle = 90;
        bool freeAxis = false;
        float weight = 1;
        bool dataValid = false;
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
        void setJointData(std::string jointName, float minAngle, float maxAngle, bool freeAxis = false, float weight = 1);
        JointData getJointData(std::string jointName);
    };

    class IKNode {
    private:
        friend class IKRig;
        IKNode() = default;
        IKNode(std::string jointName, int jointIndex, IKNode* parent=nullptr, float weight=1);
        float m_weight = 1;
        float m_minAngle;
        float m_maxAngle;
        bool m_freeAxis = false;
        std::string m_jointName;
        int m_jointIndex;
        IKNode* m_parent;
        Vector3f m_rotationAxis;
    };

    class IKRig{
        public:
            IKRig(BVHData baseAnim, RigData rigData, bool adjustFeet);
            std::vector<BVHData> m_bvhAnims;
            int m_currentClipIndex = -1;
            int m_targetClipIndex = -1;
            bool m_adjustFeet;
            EnvironmentData m_environmentData;
            std::vector<IKNode> m_nodes;
            std::pair<int, int> m_spine = { -1,-1 };
            std::pair<int, int> m_leftLeg = { -1,-1 };
            std::pair<int, int> m_rightLeg = { -1,-1 };
            std::pair<int, int> m_leftArm = { -1,-1 };
            std::pair<int, int> m_rightArm = { -1,-1 };
            std::pair<int, int> m_leftFoot = { -1,-1 };
            std::pair<int, int> m_rightFoot = { -1,-1 };
            void addAnimation(BVHData animation);
            int removeAnimation(BVHData animation);
            void setClipAnimData(std::shared_ptr<AnimationClip> clip, int firstFrame, int lastFrame);
    };

}


#endif