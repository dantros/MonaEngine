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
        void checkLegGlobalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain, glm::fquat baseRotation);
        void correctLegLocalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain);
        void checkLegsRotationAxes_base(IKRigConfig* config);
        void debugRotationAxes(IKRigConfig* config, std::vector<JointIndex> targetJoints);
    };







}

#endif