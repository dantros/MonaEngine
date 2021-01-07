#include "AnimationController.hpp"
#include "AnimationClip.hpp"
#include "Skeleton.hpp"
#include "../Core/Log.hpp"
namespace Mona {
	void BlendPoses(std::vector<JointPose>& output,
		std::vector<JointPose>& firstPose,
		std::vector<JointPose>& secondPose,
		float t)
	{
		for (uint32_t i = 0; i < output.size(); i++) {
			output[i] = mix(firstPose[i], secondPose[i], t);
		}
	}

	AnimationController::AnimationController(std::shared_ptr<AnimationClip> animation) noexcept : m_animationClipPtr(animation)
	{
		
		MONA_ASSERT(animation != nullptr, "AnimationController Error: Starting animation cannot be null.");
		auto skeleton = animation->GetSkeleton();
		m_currentPose.resize(skeleton->JointCount());
	}
	void AnimationController::PlayAnimation(std::shared_ptr<AnimationClip> animation) noexcept {
		if (m_animationClipPtr == animation)
			return;
		m_sampleTime = 0.0f;
		m_animationClipPtr = animation;
		m_crossfadeTarget.Clear();
	}

	void AnimationController::FadeTo(std::shared_ptr<AnimationClip> animation,
		BlendType blendType,
		float fadeDuration,
		float startTime) noexcept {

		if (m_animationClipPtr == animation || m_crossfadeTarget.GetAnimationClip() == animation || animation == nullptr)
			return;

		if (!m_crossfadeTarget.IsNullTarget()) {
			m_animationClipPtr = m_crossfadeTarget.m_targetClip;
			m_sampleTime = m_crossfadeTarget.m_sampleTime;
			m_isLooping = m_crossfadeTarget.m_isLooping;
			m_crossfadeTarget.Clear();
		}
		float clipDuration = m_animationClipPtr->GetDuration();
		float normalizedTimeClip = m_sampleTime / clipDuration;
		m_crossfadeTarget.SetAnimationClip(animation,
			blendType,
			fadeDuration,
			startTime,
			normalizedTimeClip);
	}

	void AnimationController::ClearFadeTo() noexcept {
		if (m_crossfadeTarget.IsNullTarget())
			return;
		m_crossfadeTarget.Clear();
	}

	void AnimationController::UpdateCurrentPose(float timeStep) noexcept {
		if (!m_crossfadeTarget.IsNullTarget()) {
			m_crossfadeTarget.m_elapsedTime += timeStep;
			if (m_crossfadeTarget.m_fadeDuration <= m_crossfadeTarget.m_elapsedTime) {
				m_animationClipPtr = m_crossfadeTarget.m_targetClip;
				m_sampleTime = m_crossfadeTarget.m_sampleTime;
				m_isLooping = m_crossfadeTarget.m_isLooping;
				m_crossfadeTarget.Clear();

			}
		}

		//Clean all the pose
		std::fill(m_currentPose.begin(), m_currentPose.end(), JointPose());
		//Sample => Local Poses


		if (!m_crossfadeTarget.IsNullTarget())
		{
			float clipDuration = m_animationClipPtr->GetDuration();
			float playbackFactorClip = 1.0f;
			float playbackFactorTarget = 1.0f;
			float factor = m_crossfadeTarget.m_elapsedTime / m_crossfadeTarget.m_fadeDuration;
			if (factor > 1.0f) factor = 1.0f;
			if (m_crossfadeTarget.m_blendType == BlendType::KeepSynchronize)
			{
				float targetDuration = m_crossfadeTarget.m_targetClip->GetDuration();
				float durationRatio = clipDuration / targetDuration;
				playbackFactorTarget = (1.0f - factor) * durationRatio + factor;
				playbackFactorClip = playbackFactorTarget * durationRatio;
			}
			else if (m_crossfadeTarget.m_blendType == BlendType::Freeze) {
				playbackFactorClip = 0.0f;
			}

			m_sampleTime = m_animationClipPtr->Sample(m_currentPose,
				m_sampleTime + timeStep * m_playRate * playbackFactorClip,
				m_isLooping);

			auto& targetPose = m_crossfadeTarget.m_currentPose;
			std::fill(targetPose.begin(), targetPose.end(), JointPose());
			m_crossfadeTarget.m_sampleTime = m_crossfadeTarget.m_targetClip->Sample(targetPose,
				m_crossfadeTarget.m_sampleTime + timeStep * m_playRate * playbackFactorTarget,
				m_crossfadeTarget.m_isLooping);
			BlendPoses(m_currentPose, m_currentPose, targetPose, factor);
		}
		else
		{
			m_sampleTime = m_animationClipPtr->Sample(m_currentPose, m_sampleTime + timeStep * m_playRate, m_isLooping);
		}


		//Travel Tree => Global Pose
		auto skeleton = m_animationClipPtr->GetSkeleton();
		for (uint32_t i = 1; i < m_currentPose.size(); i++) {
			int32_t parentIndex = skeleton->GetParentIndex(i);
			m_currentPose[i] = m_currentPose[parentIndex] * m_currentPose[i];
		}
	}

	void AnimationController::GetMatrixPalette(std::vector<glm::mat4>& outMatrixPalette) const
	{
		//Multiply by bind
		auto skeleton = m_animationClipPtr->GetSkeleton();
		auto& invBindPoseMatrices = skeleton->GetInverseBindPoseMatrices();
		for (uint32_t i = 0; i < skeleton->JointCount(); i++) {
			outMatrixPalette[i] = JointPoseToMat4(m_currentPose[i]) * invBindPoseMatrices[i];
		}
	}


}