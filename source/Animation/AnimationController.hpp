#pragma once
#ifndef ANIMATIONCONTROLLER_HPP
#define ANIMATIONCONTROLLER_HPP
#include <vector>
#include <memory>
#include "CrossFadeTarget.hpp"
#include "JointPose.hpp"
namespace Mona {
	class AnimationClip;
	class AnimationController {
		friend class AnimationSystem;
		friend class World;
	public:
		AnimationController(std::shared_ptr<AnimationClip> animation) noexcept;
		void PlayAnimation(std::shared_ptr<AnimationClip> animation) noexcept;
		void FadeTo(std::shared_ptr<AnimationClip> animation,
			BlendType blendType,
			float fadeDuration,
			float startTime) noexcept;
		void ClearFadeTo() noexcept;
		void SetIsLooping(bool value) { m_isLooping = value; }
		bool GetIsLooping() const { return m_isLooping; }
		void SetPlayRate(float playrate) { m_playRate = playrate; }
		float GetPlayRate() const { return m_playRate; }
		void GetMatrixPalette(std::vector<glm::mat4>& outMatrixPalette) const;
		std::shared_ptr<AnimationClip> GetCurrentAnimation() const { return m_animationClipPtr;  }
		JointPose GetJointModelPose(uint32_t jointIndex) const;
	private:
		void UpdateCurrentPose(float timeStep) noexcept;
		float m_sampleTime = 0.0f;
		float m_playRate = 1.0f;
		bool m_isLooping = true;
		std::vector<JointPose> m_currentPose;
		CrossFadeTarget m_crossfadeTarget;
		std::shared_ptr<AnimationClip> m_animationClipPtr;
	};
}
#endif