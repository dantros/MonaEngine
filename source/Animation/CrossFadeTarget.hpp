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
	enum class BlendType {
		Smooth,
		Freeze,
		KeepSynchronize
	};
	class CrossFadeTarget {
	public:
		CrossFadeTarget() = default;
		std::shared_ptr<AnimationClip> GetAnimationClip() const {
			return m_targetClip;
		}
		void SetAnimationClip(std::shared_ptr<AnimationClip> target,
			BlendType type,
			float fadeDuration,
			float startTime,
			float timeFactor) {
			m_blendType = type;
			m_targetClip = target;
			m_currentPose.resize(target->GetSkeleton()->JointCount(), JointPose());
			std::fill(m_currentPose.begin(), m_currentPose.end(), JointPose());
			m_fadeDuration = fadeDuration;
			m_elapsedTime = 0.0f;
			m_sampleTime = type == BlendType::KeepSynchronize? timeFactor*target->GetDuration() : startTime;
		}
		bool IsNullTarget() const { return m_targetClip == nullptr; }
		void Clear() {
			m_currentPose.clear();
			m_targetClip = nullptr;
		}
		std::vector<JointPose> m_currentPose;
		BlendType m_blendType;
		float m_fadeDuration = 1.3f;
		float m_elapsedTime = 0.0f;
		float m_sampleTime = 0.0f;
		bool m_isLooping = true;
		std::shared_ptr<AnimationClip> m_targetClip = nullptr;

		
	};
}
#endif