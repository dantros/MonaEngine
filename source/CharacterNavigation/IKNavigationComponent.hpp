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
			IKNavigationComponent(RigData& rigData, std::shared_ptr<AnimationClip> baseAnimationClip) {
				if (!rigData.isValid()) {
					MONA_LOG_ERROR("IKNavigationComponent: input rigData was not valid.");
					return;
				}
				m_rigData = rigData;
				m_baseAnimationClip = baseAnimationClip;
			}
			void AddAnimation(std::shared_ptr<AnimationClip> animationClip) {
				m_ikRig.addAnimation(animationClip, m_skeletalMeshManagerPtr);
			}

			int RemoveAnimation(std::shared_ptr<AnimationClip> animationClip) {
				return m_ikRig.removeAnimation(animationClip);
			}
			void AddTerrain(const Terrain& terrain) {
				m_ikRig.m_environmentData.addTerrain(terrain, m_staticMeshManagerPtr);
			}
			int RemoveTerrain(const Terrain& terrain) {
				return m_ikRig.m_environmentData.removeTerrain(terrain, m_staticMeshManagerPtr);
			}
		private:
			std::shared_ptr<AnimationClip> m_baseAnimationClip;
			RigData m_rigData;
			IKRig m_ikRig;
			ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
			ComponentManager<StaticMeshComponent>* m_staticMeshManagerPtr = nullptr;
			ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
			ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
	};
}






#endif