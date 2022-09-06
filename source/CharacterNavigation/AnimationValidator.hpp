#pragma once
#ifndef ANIMATIONVALIDATOR_HPP
#define ANIMATIONVALIDATOR_HPP
#include "IKRigBase.hpp"

namespace Mona{

    class IKRig;
    class AnimationValidator {
        IKRig* m_ikRig;
    public:
        AnimationValidator() = default;
        AnimationValidator(IKRig* ikRig);
        void checkTransforms(std::shared_ptr<AnimationClip> animationClip);
        void checkLegGlobalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain);
        void checkLegLocalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain, glm::vec3 originalUpVector, glm::vec3 originalFrontVector);
        void correctAnimationOrientation(std::shared_ptr<AnimationClip> animation, glm::vec3 originalUpVector, glm::vec3 originalFrontVector);
    };







}

#endif