#pragma once
#ifndef IKNAVIGATIONLIFETIMEPOLICY_HPP
#define IKNAVIGATIONLIFETIMEPOLICY_HPP
#include "IKNavigationComponent.hpp"
#include "../World/ComponentHandle.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/TransformComponent.hpp"
#include "../PhysicsCollision/RigidBodyComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "EnvironmentData.hpp"
namespace Mona {
	class IKNavigationLifetimePolicy {
	public:
		IKNavigationLifetimePolicy() = default;
		IKNavigationLifetimePolicy(ComponentManager<TransformComponent>* transformManagerPtr, ComponentManager<StaticMeshComponent>* staticMeshManagerPtr, 
			ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr, ComponentManager<IKNavigationComponent>* ikNavigationManagerPtr) : 
			m_transformManagerPtr(transformManagerPtr), m_staticMeshManagerPtr(staticMeshManagerPtr), 
			m_skeletalMeshManagerPtr(skeletalMeshManagerPtr), m_ikNavigationManagerPtr(ikNavigationManagerPtr){}

		void OnAddComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
			ikNav.m_skeletalMeshManagerPtr = m_skeletalMeshManagerPtr;
			ikNav.m_staticMeshManagerPtr = m_staticMeshManagerPtr;
			ikNav.m_transformManagerPtr = m_transformManagerPtr;

			// validar clip base
			InnerComponentHandle skeletalMeshHandle = m_ikNavigationManagerPtr->GetOwner(handle)->GetInnerComponentHandle<SkeletalMeshComponent>();
			std::shared_ptr<Skeleton> skeletonPtr = m_skeletalMeshManagerPtr->GetComponentPointer(skeletalMeshHandle)->GetSkeleton();
			AnimationController* animController = &m_skeletalMeshManagerPtr->GetComponentPointer(skeletalMeshHandle)->GetAnimationController();

			ikNav.m_ikRigController = IKRigController(animController, 
				IKRig(skeletonPtr, ikNav.m_rigData, skeletalMeshHandle));
			// setear la animacion base
			ikNav.AddAnimation(ikNav.m_baseAnimationClip);
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		ComponentManager<StaticMeshComponent>* m_staticMeshManagerPtr = nullptr;
		ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
		ComponentManager<IKNavigationComponent>* m_ikNavigationManagerPtr = nullptr;
	};
};


#endif