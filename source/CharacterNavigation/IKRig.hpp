#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP
#include <memory>
#include "../Animation/AnimationClip.hpp"
#include "BVHManager.hpp"

namespace Mona{

    class IKRig{
        public:
            IKRig();
            std::shared_ptr<AnimationClip> animationClip;
            BVHData baseData;
            BVHData currentData;

    };

    class IKNode{
        private:
            friend class IKRig;
            IKNode();


    };

}


#endif