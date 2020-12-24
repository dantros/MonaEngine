#pragma once
#ifndef SKELETALMESHCOMPONENT_H
#define SKELETALMESHCOMPONENT_H
#include <string_view>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include "../Rendering/Mesh.hpp"
#include "../Rendering/Material.hpp"
#include "../World/ComponentTypes.hpp"
#include "../Animation/Skeleton.hpp"
#include "../Animation/AnimationClip.hpp"

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
		SkeletalMeshComponent(std::shared_ptr<Mesh> mesh, std::shared_ptr<Skeleton> skeleton, std::shared_ptr<AnimationClip> animation, std::shared_ptr<Material> material)
			: m_meshPtr(mesh), m_materialPtr(material), m_skeletonPtr(skeleton), m_animationClipPtr(animation)
		{
			MONA_ASSERT(mesh != nullptr, "SkeletalMeshComponent Error: Mesh pointer cannot be null.");
			MONA_ASSERT(material != nullptr, "SkeletalMeshComponent Error: Material cannot be null.");
			MONA_ASSERT(skeleton != nullptr, "SkeletalMeshComponent Error: Skeleton cannot be null.");
			m_matrixPalette.resize(skeleton->JointCount(), glm::mat4(1.0f));
			//m_startTime = 0;
			m_elapsedTime = 0;

		}
		uint32_t GetMeshIndexCount() const noexcept {
			return m_meshPtr->GetCount();
		}

		uint32_t GetMeshVAOID() const noexcept {
			return m_meshPtr->GetVAOID();
		}

		const std::vector<glm::mat4>& GetMatrixPalette() const {
			return m_matrixPalette;
		}
		void Update(glm::mat4& worldTransform, float deltaTime) {
			m_elapsedTime += deltaTime;
			//Clean all the pose
			std::fill(m_matrixPalette.begin(), m_matrixPalette.end(), glm::mat4(1.0f));
			
			//Sample => Local Poses
			m_animationClipPtr->Sample(m_matrixPalette, m_elapsedTime);
			//Travel Tree => Global Pose
			
			for (uint32_t i = 1; i < m_matrixPalette.size(); i++) {
				int32_t parentIndex = m_skeletonPtr->GetParentIndex(i);
				m_matrixPalette[i] = m_matrixPalette[parentIndex] * m_matrixPalette[i];
			}
			//Multiply by bind
			
			for (uint32_t i = 0; i < m_matrixPalette.size(); i++) {
				m_matrixPalette[i] = m_matrixPalette[i] * m_skeletonPtr->GetInverseBindPoseMatrices()[i];
			}

			
			
		}
		
	private:
		//float m_startTime = 0.0f;
		float m_elapsedTime = 0.0f;
		/*
		uint32_t m_currentRotationIndex = 0;
		uint32_t m_currentPositionIndex = 0;
		uint32_t m_currentScaleIndex = 0;
		*/
		std::shared_ptr<AnimationClip> m_animationClipPtr;
		std::shared_ptr<Mesh> m_meshPtr;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		std::shared_ptr<Material> m_materialPtr;
		std::vector<glm::mat4> m_matrixPalette;
		//std::vector<JointPose> m_currentPose;
	};
}
#endif