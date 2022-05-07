#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP
#include <memory>
#include "BVHManager.hpp"

namespace Mona{
    struct ChainData {
        std::string startJointName;
        std::string endJointName;
    };
    struct RigData {
        ChainData spine;
        ChainData leftLeg;
        ChainData rightLeg;
        ChainData leftArm;
        ChainData rightArm;
        ChainData leftFoot;
        ChainData rightFoot;
        std::vector<std::pair<std::string, float>> weightModifiers;
    };
    class IKRig{
        public:
            IKRig(std::vector<BVHData*> m_bvhAnims, RigData rigData, bool adjustFeet);
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