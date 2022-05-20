#pragma once
#ifndef IKNAVIGATIONCOMPONENT_HPP
#define IKNAVIGATIONCOMPONENT_HPP

#include <string_view>
#include "../World/ComponentHandle.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "IKRig.hpp"
namespace Mona {
	class IKNavigationLifetimePolicy;
	class RigData;
	class IKNavigationComponent{
		public:
			friend class IKNavigationLifetimePolicy;
			using LifetimePolicyType = IKNavigationLifetimePolicy;
			using dependencies = DependencyList<SkeletalMeshComponent, RigidBodyComponent>;
			static constexpr std::string_view componentName = "IKNavigationComponent";
			static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::IKNavigationComponent);
			IKNavigationComponent(const RigData& rigData, std::shared_ptr<AnimationClip> baseAnimation, bool adjustFeet) {
				m_rigData = rigData;
				m_animationClips.push_back(baseAnimation);
				m_adjustFeet = adjustFeet;
			}
			void AddAnimation(std::shared_ptr<AnimationClip> animationClip) {
				std::shared_ptr<Skeleton> skeletonPtr = m_skeletalMeshManagerPtr->GetComponentPointer(m_skeletalMeshHandle)->GetSkeleton();
				if (animationClip->GetSkeleton() != skeletonPtr) {
					MONA_LOG_ERROR("Input animation does not correspond to base skeleton.");
					return;
				}
				std::shared_ptr<BVHData> bvhPtr = BVHManager::GetInstance().readBVH(animationClip);
				m_ikRig.addAnimation(bvhPtr);
			}

			int RemoveAnimation(std::shared_ptr<AnimationClip> animationClip) {
				std::shared_ptr<BVHData> bvhPtr = BVHManager::GetInstance().getBVHData(animationClip);
				if ( bvhPtr != nullptr) {
					return m_ikRig.removeAnimation(bvhPtr);
				}
				return -1;
			}
			void AddTerrain(const Terrain& terrain) {
				m_ikRig.m_environmentData.addTerrain(terrain, m_staticMeshManagerPtr);
			}
			int RemoveTerrain(const Terrain& terrain) {
				return m_ikRig.m_environmentData.removeTerrain(terrain, m_staticMeshManagerPtr);
			}
		private:
			std::vector<std::shared_ptr<AnimationClip>> m_animationClips;
			InnerComponentHandle m_skeletalMeshHandle;
			RigData m_rigData;
			IKRig m_ikRig;
			bool m_adjustFeet;
			ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
			ComponentManager<StaticMeshComponent>* m_staticMeshManagerPtr = nullptr;
			ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
			ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
			ComponentManager<IKNavigationComponent>* m_ikNavigationManagerPtr = nullptr;
	};
}






#endif