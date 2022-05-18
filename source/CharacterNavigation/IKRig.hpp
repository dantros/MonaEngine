#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include <memory>
#include "BVHManager.hpp"

namespace Mona{
    typedef Eigen::Matrix<float, 1, 3> Vector3f;
    typedef Eigen::Matrix<float, 1, 2> Vector2f;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;

    struct JointData {
        float minAngle;
        float maxAngle;
        bool freeAxis = false;
        float weightModifier = 1;
    };
    struct ChainEnds {
        std::string startJointName;
        std::string endEffectorName;
    };
    struct RigData {
        ChainEnds spine;
        ChainEnds leftLeg;
        ChainEnds rightLeg;
        ChainEnds leftArm;
        ChainEnds rightArm;
        ChainEnds leftFoot;
        ChainEnds rightFoot;

        std::unordered_map<std::string, JointData> jointData;
        void setJointData(std::string jointName, float minAngle, float maxAngle, bool freeAxis = false, float weightModifier = 1) {
            jointData[jointName] = JointData(minAngle, maxAngle, freeAxis, weightModifier);
        }
    };

    class IKNode {
    private:
        friend class IKRig;
        IKNode();
        IKNode(std::string jointName, int jointIndex, IKNode* parent=nullptr, float weight=1);
        float m_weight = 1;
        bool m_freeAxis = false;
        std::string m_jointName;
        int m_jointIndex;
        IKNode* m_parent;
        Vector3f m_rotationAxis;
    };

    class IKRig{
        public:
            IKRig(std::vector<BVHData*> bvhAnims, RigData rigData, bool adjustFeet, AnimationController* animationController);
            AnimationController* m_animationController;
            std::vector<BVHData*> m_bvhAnims;
            int m_currentClipIndex = -1;
            int m_targetClipIndex = -1;
            bool m_adjustFeet;
            std::vector<IKNode> m_nodes;
            std::pair<IKNode*, IKNode*> m_spine;
            std::pair<IKNode*, IKNode*> m_leftLeg;
            std::pair<IKNode*, IKNode*> m_rightLeg;
            std::pair<IKNode*, IKNode*> m_leftArm;
            std::pair<IKNode*, IKNode*> m_rightArm;
            std::pair<IKNode*, IKNode*> m_leftFoot;
            std::pair<IKNode*, IKNode*> m_rightFoot;
            void setClipAnimData(std::shared_ptr<AnimationClip> clip, int firstFrame, int lastFrame);
    };

}


#endif