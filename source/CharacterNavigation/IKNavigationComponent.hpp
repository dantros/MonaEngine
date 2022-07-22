#pragma once
#ifndef IKNAVIGATIONCOMPONENT_HPP
#define IKNAVIGATIONCOMPONENT_HPP

#include <string_view>
#include "../World/ComponentTypes.hpp"
#include "IKRigController.hpp"
namespace Mona {
	class IKNavigationLifetimePolicy;
	class RigData;
	class IKNavigationComponent{
		public:
			friend class IKNavigationLifetimePolicy;
			using LifetimePolicyType = IKNavigationLifetimePolicy;
			using dependencies = DependencyList<SkeletalMeshComponent>;
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
				m_ikRigController.addAnimation(animationClip);
			}

			int RemoveAnimation(std::shared_ptr<AnimationClip> animationClip) {
				return m_ikRigController.removeAnimation(animationClip);
			}
			void AddTerrain(const GameObjectHandle<GameObject>& staticMeshObject) {
				m_ikRigController.m_ikRig.m_trajectoryGenerator.m_environmentData.addTerrain(staticMeshObject);
			}
			int RemoveTerrain(const GameObjectHandle<GameObject>& staticMeshObject) {
				return m_ikRigController.m_ikRig.m_trajectoryGenerator.m_environmentData.removeTerrain(staticMeshObject);
			}
			IKRigController& GetIKRigController() { return m_ikRigController; }
		private:
			std::shared_ptr<AnimationClip> m_baseAnimationClip;
			RigData m_rigData;
			IKRigController m_ikRigController;
	};
}






#endif