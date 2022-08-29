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
		friend class IKRigConfig;
		friend class IKRig;
		friend class IKRigController;
		friend class TrajectoryGenerator;
		typedef int JointIndex;
		typedef int FrameIndex;
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
		void Rotate(glm::fquat rotation);
		void Scale(float scale);
		void Translate(glm::vec3 translation);
	private:
		void SetSkeleton(std::shared_ptr<Skeleton> skeletonPtr);
		AnimationClip(const std::string& filePath,
			std::shared_ptr<Skeleton> skeleton,
			bool removeRootMotion = true);
		void RemoveRootMotion();
		void RemoveJointTranslation(int jointIndex);
		void RemoveJointRotation(int jointIndex);
		void RemoveJointScaling(int jointIndex);
		void DecompressRotations();

		float GetSamplingTime(float time, bool isLooping) const;
		std::pair<uint32_t, float> GetTimeFraction(const std::vector<float>& timeStamps, float time) const;
		glm::vec3 GetPosition(float time, int joint, bool isLooping);
		glm::fquat GetRotation(float time, int joint, bool isLooping);
		glm::vec3 GetScale(float time, int joint, bool isLooping);
		void SetRotation(glm::fquat newRotation, int frameIndex, int joint);
		int GetTrackIndex(int jointIndex);

		std::vector<AnimationTrack> m_animationTracks;
		std::vector<std::string> m_trackJointNames;
		std::vector<JointIndex> m_trackJointIndices;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		float m_duration = 1.0f;
		std::string m_animationName;
	};
}
#endif