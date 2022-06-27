#pragma once
#ifndef ANIMATIONCLIP_HPP
#define ANIMATIONCLIP_HPP
#include <vector>
#include <string>
#include <memory>
#include <utility>
#include <glm/glm.hpp>
#include <Eigen/Geometry>
#include "JointPose.hpp"
namespace Mona {
	class Skeleton;
	class AnimationClip {
	public:
		friend class AnimationClipManager;
		friend class IKRig;
		friend class IKRigConfig;
		friend class IKRigController;
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
		std::string GetAnimationName() {
			return m_animationName;
		}
		
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
		glm::vec3 GetPosition(float time, int joint, bool isLooping);
		glm::fquat GetRotation(float time, int joint, bool isLooping);
		glm::vec3 GetScale(float time, int joint, bool isLooping);
		void SetNearestRotation(glm::fquat newRotation, float time, int joint, bool isLooping);
		void AddRotation(glm::fquat newRotation, float time, int joint, bool isLooping);

		std::vector<AnimationTrack> m_animationTracks;
		std::vector<std::string> m_trackJointNames;
		std::vector<jointIndex> m_trackJointIndices;
		std::vector<int> m_jointTrackIndices;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		float m_duration = 1.0f;
		std::string m_animationName;
		bool m_stableRotations = false;
	};
}
#endif