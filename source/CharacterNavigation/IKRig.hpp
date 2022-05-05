#pragma once
#ifndef IKRIG_HPP
#define IKRIG_HPP
#include <memory>
#include "../Animation/AnimationController.hpp"
#include "BVHManager.hpp"

namespace Mona{

    class IKRig{
        public:
            IKRig();
            AnimationController m_animationController;
            BVHData m_bvhData;
            std::vector<VectorXf> getClipRootPositions(std::shared_ptr<AnimationClip> clip, int frame);
            std::vector<MatrixXf> getClipRotations(std::shared_ptr<AnimationClip> clip, int frame);
            void setClipAnimData(std::shared_ptr<AnimationClip> clip, int firstFrame, int lastFrame);
    };

    class IKNode{
        private:
            friend class IKRig;
            IKNode();


    };

}


#endif