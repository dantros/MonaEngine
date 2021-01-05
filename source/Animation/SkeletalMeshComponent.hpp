#pragma once
#ifndef SKELETALMESHCOMPONENT_H
#define SKELETALMESHCOMPONENT_H
#include <string_view>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "../Animation/SkinnedMesh.hpp"
#include "../Rendering/Material.hpp"
#include "../World/ComponentTypes.hpp"
#include "Skeleton.hpp"
#include "AnimationClip.hpp"
#include "JointPose.hpp"
#include "CrossFadeTarget.hpp"

namespace Mona {
	inline void BlendPoses(std::vector<JointPose>& output,
		std::vector<JointPose>& firstPose,
		std::vector<JointPose>& secondPose,
		float t)
	{
		for (uint32_t i = 0; i < output.size(); i++) {
			output[i] = mix(firstPose[i], secondPose[i], t);
		}
	}
	class TransformComponent;
	class SkeletalMeshComponent
	{
	public:
		friend class Renderer;
		friend class World;
		using managerType = ComponentManager<SkeletalMeshComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "SkeletalMeshComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::SkeletalMeshComponent);

		SkeletalMeshComponent(std::shared_ptr<SkinnedMesh> skinnedMesh, std::shared_ptr<AnimationClip> animation, std::shared_ptr<Material> material)
			: m_skinnedMeshPtr(skinnedMesh), m_materialPtr(material), m_animationClipPtr(animation), m_crossfadeTarget()
		{
			MONA_ASSERT(skinnedMesh != nullptr, "SkeletalMeshComponent Error: Mesh pointer cannot be null.");
			MONA_ASSERT(material != nullptr, "SkeletalMeshComponent Error: Material cannot be null.");
			//MONA_ASSERT(skeleton != nullptr, "SkeletalMeshComponent Error: Skeleton cannot be null.");
			auto skeleton = skinnedMesh->GetSkeleton();
			m_currentPose.resize(skeleton->JointCount());
			//m_startTime = 0;
			m_elapsedTime = 0;

		}
		uint32_t GetMeshIndexCount() const noexcept {
			return m_skinnedMeshPtr->GetCount();
		}

		uint32_t GetMeshVAOID() const noexcept {
			return m_skinnedMeshPtr->GetVAOID();
		}

		uint32_t GetJointCount() const noexcept {
			return m_currentPose.size();
		}
		void GetMatrixPalette(std::vector<glm::mat4>& outMatrixPalette) const {
			//Multiply by bind
			auto skeleton = m_skinnedMeshPtr->GetSkeleton();
			auto& invBindPoseMatrices = skeleton->GetInverseBindPoseMatrices();
			for (uint32_t i = 0; i < skeleton->JointCount(); i++) {
				outMatrixPalette[i] = JointPoseToMat4(m_currentPose[i]) * invBindPoseMatrices[i];
			}
		}
		
		void PlayAnimation(std::shared_ptr<AnimationClip> animation) {
			if (m_animationClipPtr == animation)
				return;
			m_elapsedTime = 0.0f;
			m_animationClipPtr = animation;
			m_crossfadeTarget.Clear();
		}

		void FadeTo(std::shared_ptr<AnimationClip> animation)
		{
			if (m_animationClipPtr == animation || m_crossfadeTarget.GetAnimationClip() == animation)
				return;
			m_crossfadeTarget.Clear();
			m_crossfadeTarget.SetAnimationClip(animation);
		}

		void SetIsLooping(bool value) {
			m_isLooping = value;
		}

		bool GetIsLooping() const {
			return m_isLooping;
		}
	private:
		void Update(glm::mat4& worldTransform, float deltaTime) {
			m_elapsedTime += deltaTime;
			if (!m_crossfadeTarget.IsNullTarget()) {
				m_crossfadeTarget.m_elapsedTime += deltaTime;
				if (m_crossfadeTarget.m_fadeDuration <= m_crossfadeTarget.m_elapsedTime) {
					m_animationClipPtr = m_crossfadeTarget.m_targetClip;
					m_elapsedTime = m_crossfadeTarget.m_startTime + m_crossfadeTarget.m_elapsedTime;
					m_isLooping = m_crossfadeTarget.m_isLooping;
					m_crossfadeTarget.Clear();

				}
			}
			
			//Clean all the pose
			std::fill(m_currentPose.begin(), m_currentPose.end(), JointPose());
			//Sample => Local Poses

			m_animationClipPtr->Sample(m_currentPose, m_elapsedTime, m_isLooping);
			if (!m_crossfadeTarget.IsNullTarget()) {
				auto& targetPose = m_crossfadeTarget.m_currentPose;
				std::fill(targetPose.begin(), targetPose.end(), JointPose());
				float time = m_crossfadeTarget.m_startTime + m_crossfadeTarget.m_elapsedTime;
				m_crossfadeTarget.m_targetClip->Sample(targetPose, time, m_crossfadeTarget.m_isLooping);
				float factor = m_crossfadeTarget.m_elapsedTime / m_crossfadeTarget.m_fadeDuration;
				BlendPoses(m_currentPose, m_currentPose, targetPose, factor);
			}
			//Travel Tree => Global Pose
			auto skeleton = m_skinnedMeshPtr->GetSkeleton();
			for (uint32_t i = 1; i < m_currentPose.size(); i++) {
				int32_t parentIndex = skeleton->GetParentIndex(i);
				m_currentPose[i] = m_currentPose[parentIndex] * m_currentPose[i];
			}


		}

		float m_elapsedTime = 0.0f;
		bool m_isLooping = true;
		CrossFadeTarget m_crossfadeTarget;
		std::shared_ptr<AnimationClip> m_animationClipPtr;
		std::shared_ptr<SkinnedMesh> m_skinnedMeshPtr;
		std::shared_ptr<Material> m_materialPtr;
		std::vector<JointPose> m_currentPose;
	};
}
#endif