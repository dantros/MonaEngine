#pragma once
#ifndef ANIMATIONCLIP_HPP
#define ANIMATIONCLIP_HPP
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <glm/glm.hpp>
#include <math.h>
#include <algorithm>
#include "../Core/Log.hpp"
#include "JointPose.hpp"
#include "Skeleton.hpp"
namespace Mona {
	class AnimationClip {
	public:
		friend class AnimationClipManager;
		using jointIndex = uint32_t;
		struct AnimationTrack {
			std::vector<glm::vec3> positions;
			std::vector<glm::fquat> rotations;
			std::vector<glm::vec3> scales;
			std::vector<float> positionTimeStamps;
			std::vector<float> rotationTimeStamps;
			std::vector<float> scaleTimeStamps;

		};
	private:
		AnimationClip(std::vector<AnimationTrack>&& animationTracks,
			std::vector<std::string>&& trackNames,
			std::shared_ptr<Skeleton> skeletonPtr,
			float duration,
			float ticksPerSecond) :
			m_animationTracks(std::move(animationTracks)),
			m_trackNames(std::move(trackNames)),
			m_skeletonPtr(nullptr),
			m_duration(duration),
			m_ticksPerSecond(ticksPerSecond)
		{
			m_jointIndices.resize(m_trackNames.size());
			SetSkeleton(skeletonPtr);
		}

		void SetSkeleton(std::shared_ptr<Skeleton> skeletonPtr) {
			if (!skeletonPtr || skeletonPtr == m_skeletonPtr)
			{
				return;
			}
			for (uint32_t i = 0; i < m_trackNames.size(); i++) {
				const std::string& name = m_trackNames[i];
				int32_t signIndex = skeletonPtr->GetJointIndex(name);
				MONA_ASSERT(signIndex >= 0, "AnimationClip Error: Given skeleton incompatible with importing animation");
				uint32_t jointIndex = static_cast<uint32_t>(signIndex);
				m_jointIndices[i] = jointIndex;
			}
			m_skeletonPtr = skeletonPtr;
		}
		float GetSamplingTime(float time) const {
			if (m_isLooping)
				return std::fmod(time, m_duration);
			return std::clamp(time, 0.0f, m_duration);
		}
		std::pair<uint32_t, float> GetTimeFraction(const std::vector<float>& timeStamps, float time) const {
			uint32_t sample = 0;
			while (time >= timeStamps[sample]) {
				sample++;
			}
			float start = timeStamps[sample - 1];
			float end = timeStamps[sample];
			float frac = (time - start) / (end - start);
			return { sample, frac };
		}
	public:
		void Sample(std::vector<JointPose>& outPose, float time) {
			float newTime = GetSamplingTime(time);
			for (uint32_t i = 0; i < m_animationTracks.size(); i++)
			{
				const AnimationTrack& animationTrack = m_animationTracks[i];
				uint32_t jointIndex = m_jointIndices[i];
				std::pair<uint32_t, float> fp;
				glm::vec3 localPosition;
				if (animationTrack.positions.size() > 1)
				{
					fp = GetTimeFraction(animationTrack.positionTimeStamps, newTime);
					const glm::vec3& position = animationTrack.positions[fp.first - 1];
					const glm::vec3& nextPosition = animationTrack.positions[fp.first];
					localPosition = glm::mix(position, nextPosition, fp.second);
				}
				else {
					localPosition = animationTrack.positions[0];
				}
				
				glm::fquat localRotation;
				if (animationTrack.rotations.size() > 1)
				{
					fp = GetTimeFraction(animationTrack.rotationTimeStamps, newTime);
					const glm::fquat& rotation = animationTrack.rotations[fp.first - 1];
					const glm::fquat& nextRotation = animationTrack.rotations[fp.first];
					localRotation = glm::slerp(rotation, nextRotation, fp.second);
				}
				else {
					localRotation = animationTrack.rotations[0];
				}

				glm::vec3 localScale;
				if (animationTrack.scales.size() > 1) {
					fp = GetTimeFraction(animationTrack.scaleTimeStamps, newTime);
					const glm::vec3& scale = animationTrack.scales[fp.first - 1];
					const glm::vec3& nextScale = animationTrack.scales[fp.first];
					localScale = glm::mix(scale, nextScale, fp.second);
				}
				else {
					localScale = animationTrack.scales[0];
				}

				outPose[jointIndex] = JointPose(localRotation, localPosition, localScale);

			}
		}
	private:
		std::vector<AnimationTrack> m_animationTracks;
		std::vector<std::string> m_trackNames;
		std::vector<jointIndex> m_jointIndices;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		float m_duration = 1.0f;
		float m_ticksPerSecond = 1.0f;
		bool m_isLooping = true;
	};
}
#endif