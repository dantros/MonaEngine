#pragma once
#ifndef CROSSFADETARGET_HPP
#define CROSSFADETARGET_HPP
#include <memory>
#include <vector>
#include <algorithm>
#include "AnimationClip.hpp"
#include "JointPose.hpp"
#include "Skeleton.hpp"
namespace Mona {
	class CrossFadeTarget {
	public:
		CrossFadeTarget() = default;
		std::shared_ptr<AnimationClip> GetAnimationClip() const {
			return m_targetClip;
		}
		void SetAnimationClip(std::shared_ptr<AnimationClip> animationClip){
			m_targetClip = animationClip;
			m_currentPose.resize(animationClip->GetSkeleton()->JointCount(), JointPose());
			std::fill(m_currentPose.begin(), m_currentPose.end(), JointPose());
			m_fadeDuration = 0.3f;
			m_elapsedTime = 0.0f;
			m_startTime = 0.0f;
		}
		bool IsNullTarget() const { return m_targetClip == nullptr; }
		void Clear() {
			m_currentPose.clear();
			m_targetClip = nullptr;
		}
		std::vector<JointPose> m_currentPose;
		float m_fadeDuration = 1.3f;
		float m_elapsedTime = 0.0f;
		float m_startTime = 0.0f;
		bool m_isLooping = true;
		std::shared_ptr<AnimationClip> m_targetClip = nullptr;

		
	};
}
#endif