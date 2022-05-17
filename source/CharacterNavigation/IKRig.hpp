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
    struct ChainData {
        std::string startJointName;
        std::string endEffectorName;
    };
    struct RigData {
        ChainData spine;
        ChainData leftLeg;
        ChainData rightLeg;
        ChainData leftArm;
        ChainData rightArm;
        ChainData leftFoot;
        ChainData rightFoot;

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
        std::string m_jointName;
        int m_jointIndex;
        IKNode* m_parent;
        std::vector<IKNode*> m_children;
        Vector3f m_rotationAxis;
        float m_rotationAngle;
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
            std::vector<IKNode*> m_spineChain;
            std::vector<IKNode*> m_leftLegChain;
            std::vector<IKNode*> m_rightLegChain;
            std::vector<IKNode*> m_leftArmChain;
            std::vector<IKNode*> m_rightArmChain;
            std::vector<IKNode*> m_leftFootChain;
            std::vector<IKNode*> m_rightFootChain;
            void setClipAnimData(std::shared_ptr<AnimationClip> clip, int firstFrame, int lastFrame);
    };

}


#endif