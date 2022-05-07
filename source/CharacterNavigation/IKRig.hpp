#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP
#include <memory>
#include "BVHManager.hpp"

namespace Mona{

    class IKRig{
        public:
            IKRig();
            AnimationController m_animationController;
            std::vector<BVHData*> bvhAnims;
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