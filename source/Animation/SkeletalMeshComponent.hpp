#pragma once
#ifndef SKELETALMESHCOMPONENT_H
#define SKELETALMESHCOMPONENT_H
#include <string_view>
#include <memory>
#include <vector>
#include "../Core/Log.hpp"
#include "../World/ComponentTypes.hpp"
#include "AnimationController.hpp"
#include "../Rendering/Material.hpp"
namespace Mona {
	class TransformComponent;
	class Skeleton;
	class Material;
	class AnimationClip;
	class SkinnedMesh;
	class SkeletalMeshComponent
	{
	public:
		friend class Renderer;
		friend class World;
		friend class AnimationSystem;
		//using managerType = ComponentManager<SkeletalMeshComponent>;
		using LifetimePolicyType = DefaultLifetimePolicy<SkeletalMeshComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "SkeletalMeshComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::SkeletalMeshComponent);

		SkeletalMeshComponent(std::shared_ptr<SkinnedMesh> skinnedMesh, std::shared_ptr<AnimationClip> animation, std::shared_ptr<Material> material)
			: m_skinnedMeshPtr(skinnedMesh), m_materialPtr(material), m_animationController(animation), m_skeleton(animation->GetSkeleton())
		{
			MONA_ASSERT(skinnedMesh != nullptr, "SkeletalMeshComponent Error: Mesh pointer cannot be null.");
			MONA_ASSERT(material != nullptr, "SkeletalMeshComponent Error: Material cannot be null.");
			MONA_ASSERT(material->IsForSkinning(), "SkeletalMeshComponent Error: Material cannot be used for this type of Mesh");
		}
		
		std::shared_ptr<Material> GetMaterial() const noexcept {
			return m_materialPtr;
		}

		void SetMaterial(std::shared_ptr<Material> material) noexcept {
			if (material != nullptr)
			{
				MONA_ASSERT(material->IsForSkinning(), "SkeletalMeshComponent Error: Material cannot be used for this type of Mesh");
				m_materialPtr = material;
			}
				
		}
		std::shared_ptr<Skeleton> GetSkeleton() const noexcept {
			return m_skeleton;
		}

		const AnimationController& GetAnimationController() const
		{
			return m_animationController;
		}
		AnimationController& GetAnimationController() {
			return m_animationController;
		}
		
		
	private:
		
		std::shared_ptr<Skeleton> m_skeleton;
		std::shared_ptr<SkinnedMesh> m_skinnedMeshPtr;
		std::shared_ptr<Material> m_materialPtr;
		AnimationController m_animationController;
	};
}
#endif