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
		float GetDuration() const { return m_duration; }
		float Sample(std::vector<JointPose>& outPose, float time, bool isLooping);
		
		std::shared_ptr<Skeleton> GetSkeleton() const {
			return m_skeletonPtr;
		}
	private:
		void SetSkeleton(std::shared_ptr<Skeleton> skeletonPtr);
		AnimationClip(const std::string& filePath,
			std::shared_ptr<Skeleton> skeleton,
			bool removeRootMotion = true);
		void RemoveRootMotion();

		float GetSamplingTime(float time, bool isLooping) const;
		std::pair<uint32_t, float> GetTimeFraction(const std::vector<float>& timeStamps, float time) const;

		std::vector<AnimationTrack> m_animationTracks;
		std::vector<std::string> m_trackJointNames;
		std::vector<jointIndex> m_trackJointIndices;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		float m_duration = 1.0f;
	};
}
#endif