#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP

#include <memory>
#include "BVHManager.hpp"

namespace Mona{
    typedef Eigen::Matrix<float, 1, 3> Vector3f;
    typedef Eigen::Matrix<float, 1, 2> Vector2f;
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;

    struct ChainData {
        std::string startJointName;
        std::string endEffectorName;
        std::vector<std::pair<std::string, float>> weightModifiers;
    };
    struct RigData {
        ChainData spine;
        ChainData leftLeg;
        ChainData rightLeg;
        ChainData leftArm;
        ChainData rightArm;
        ChainData leftFoot;
        ChainData rightFoot;
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
        Vector3f rotationAxis;
        float rotationAngle;
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
            IKNode* m_spineEE;
            IKNode* m_leftLegEE;
            IKNode* m_rightLegEE;
            IKNode* m_leftArmEE;
            IKNode* m_rightArmEE;
            IKNode* m_leftFootEE;
            IKNode* m_rightFootEE;
            void setClipAnimData(std::shared_ptr<AnimationClip> clip, int firstFrame, int lastFrame);
    };

}


#endif