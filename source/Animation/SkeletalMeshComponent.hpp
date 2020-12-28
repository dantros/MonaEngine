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


namespace Mona {
	inline void DebugGlmMatrix(const glm::mat4& mat)
	{
		MONA_LOG_INFO("\t{0} \t{1} \t{2} \t{3}", mat[0][0], mat[0][1], mat[0][2], mat[0][3]);
		MONA_LOG_INFO("\t{0} \t{1} \t{2} \t{3}", mat[1][0], mat[1][1], mat[1][2], mat[1][3]);
		MONA_LOG_INFO("\t{0} \t{1} \t{2} \t{3}", mat[2][0], mat[2][1], mat[2][2], mat[2][3]);
		MONA_LOG_INFO("\t{0} \t{1} \t{2} \t{3}", mat[3][0], mat[3][1], mat[3][2], mat[3][3]);
	}
	class TransformComponent;
	class SkeletalMeshComponent
	{
	public:
		friend class Renderer;
		using managerType = ComponentManager<SkeletalMeshComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "SkeletalMeshComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::SkeletalMeshComponent);
		SkeletalMeshComponent(std::shared_ptr<SkinnedMesh> skinnedMesh, std::shared_ptr<AnimationClip> animation, std::shared_ptr<Material> material)
			: m_skinnedMeshPtr(skinnedMesh), m_materialPtr(material), m_animationClipPtr(animation)
		{
			MONA_ASSERT(skinnedMesh != nullptr, "SkeletalMeshComponent Error: Mesh pointer cannot be null.");
			MONA_ASSERT(material != nullptr, "SkeletalMeshComponent Error: Material cannot be null.");
			//MONA_ASSERT(skeleton != nullptr, "SkeletalMeshComponent Error: Skeleton cannot be null.");
			auto skeleton = skinnedMesh->GetSkeleton();
			m_matrixPalette.resize(skeleton->JointCount(), glm::mat4(1.0f));
			m_globalPose.resize(skeleton->JointCount());
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
			return m_globalPose.size();
		}
		void GetMatrixPalette(std::vector<glm::mat4>& outMatrixPalette) const {
			//Multiply by bind
			auto skeleton = m_skinnedMeshPtr->GetSkeleton();
			auto& invBindPoseMatrices = skeleton->GetInverseBindPoseMatrices();
			for (uint32_t i = 0; i < skeleton->JointCount(); i++) {
				outMatrixPalette[i] = JointPoseToMat4(m_globalPose[i]) * invBindPoseMatrices[i];
			}
		}
		void Update(glm::mat4& worldTransform, float deltaTime) {
			m_elapsedTime += deltaTime;
			//Clean all the pose
			//std::fill(m_matrixPalette.begin(), m_matrixPalette.end(), glm::mat4(1.0f));
			std::fill(m_globalPose.begin(), m_globalPose.end(), JointPose());
			//Sample => Local Poses
			m_animationClipPtr->Sample(m_globalPose, m_elapsedTime);
			//Travel Tree => Global Pose
			auto skeleton = m_skinnedMeshPtr->GetSkeleton();
			for (uint32_t i = 1; i < m_globalPose.size(); i++) {
				int32_t parentIndex = skeleton->GetParentIndex(i);
				m_globalPose[i] = m_globalPose[parentIndex] * m_globalPose[i];
			}
			

			
			
		}
		
	private:
		//float m_startTime = 0.0f;
		float m_elapsedTime = 0.0f;
		std::shared_ptr<AnimationClip> m_animationClipPtr;
		std::shared_ptr<SkinnedMesh> m_skinnedMeshPtr;
		std::shared_ptr<Material> m_materialPtr;
		std::vector<glm::mat4> m_matrixPalette;
		std::vector<JointPose> m_globalPose;
	};
}
#endif