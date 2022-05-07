#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP
#include <memory>
#include "BVHManager.hpp"

namespace Mona{
    struct RigData {
        std::string spineStartJointName;
        std::string spineEndJointName;
        std::string leftLegStartJointName;
        std::string leftLegEndJointName;
        std::string rightLegStartJointName;
        std::string rightLegEndJointName;
        std::string leftArmStartJointName;
        std::string rightArmEndJointName;
        bool adjustFeet_IK;
        std::string leftFootStartJointName;
        std::string rightFootEndJointName;
        std::vector<std::pair<std::string, float>> weightModifiers;
    };
    class IKRig{
        public:
            IKRig(std::vector<BVHData*> m_bvhAnims, RigData rigData);
            AnimationController m_animationController;
            std::vector<BVHData*> m_bvhAnims;
            int currentClipIndex;
            int targetClipIndex;
            void setClipAnimData(std::shared_ptr<AnimationClip> clip, int firstFrame, int lastFrame);
    };

    class IKNode{
        private:
            friend class IKRig;
            IKNode();


    };

}


#endif