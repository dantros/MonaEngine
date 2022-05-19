#pragma once
#ifndef IKNAVIGATIONCOMPONENT_HPP
#define IKNAVIGATIONCOMPONENT_HPP

#include <string_view>
#include "../World/ComponentHandle.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "EnvironmentData.hpp"
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
			IKNavigationComponent(const RigData& rigData, std::shared_ptr<AnimationClip> baseAnimation);
			void AddAnimation(std::shared_ptr<AnimationClip> animationClip) {

			}
			void AddTerrain(const Terrain& terrain) {
				m_environmentData.addTerrain(terrain);
			}
		private:
			std::vector<std::shared_ptr<AnimationClip>> m_animationClips;
			EnvironmentData m_environmentData;
			IKRig m_ikRig;
			ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
			ComponentManager<StaticMeshComponent>* m_staticMeshManagerPtr = nullptr;
			ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
			ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
	};
}






#endif