#pragma once
#ifndef ANIMATIONCLIP_HPP
#define ANIMATIONCLIP_HPP
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <glm/glm.hpp>
#include "JointPose.hpp"
namespace Mona {
	class Skeleton;
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

		void Sample(std::vector<JointPose>& outPose, float time, bool isLooping);
		void SetSkeleton(std::shared_ptr<Skeleton> skeletonPtr);
		std::shared_ptr<Skeleton> GetSkeleton() const {
			return m_skeletonPtr;
		}
	private:
		AnimationClip(std::vector<AnimationTrack>&& animationTracks,
			std::vector<std::string>&& trackNames,
			std::shared_ptr<Skeleton> skeletonPtr,
			float duration,
			float ticksPerSecond,
			bool removeRootMotion);
		void RemoveRootMotion();

		float GetSamplingTime(float time, bool isLooping) const;
		std::pair<uint32_t, float> GetTimeFraction(const std::vector<float>& timeStamps, float time) const;

		std::vector<AnimationTrack> m_animationTracks;
		std::vector<std::string> m_trackNames;
		std::vector<jointIndex> m_jointIndices;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		float m_duration = 1.0f;
		float m_ticksPerSecond = 1.0f;
	};
}
#endif