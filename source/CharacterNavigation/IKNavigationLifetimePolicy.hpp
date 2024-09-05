#pragma once
#ifndef IKNAVIGATIONLIFETIMEPOLICY_HPP
#define IKNAVIGATIONLIFETIMEPOLICY_HPP

#include "IKNavigationComponent.hpp"

namespace Mona {
	class IKNavigationLifetimePolicy {
	public:
		IKNavigationLifetimePolicy() = default;
		IKNavigationLifetimePolicy(ComponentManager<TransformComponent>* transformManagerPtr, 
			ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr, ComponentManager<IKNavigationComponent>* ikNavigationManagerPtr) : 
			m_transformManagerPtr(transformManagerPtr),	m_skeletalMeshManagerPtr(skeletalMeshManagerPtr), m_ikNavigationManagerPtr(ikNavigationManagerPtr){}

		void OnAddComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {

			InnerComponentHandle skeletalMeshHandle = m_ikNavigationManagerPtr->GetOwner(handle)->GetInnerComponentHandle<SkeletalMeshComponent>();
			std::shared_ptr<Skeleton> skeletonPtr = m_skeletalMeshManagerPtr->GetComponentPointer(skeletalMeshHandle)->GetSkeleton();

			InnerComponentHandle transformHandle = m_ikNavigationManagerPtr->GetOwner(handle)->GetInnerComponentHandle<TransformComponent>();

			ikNav.m_ikRigController = IKRigController(skeletonPtr, ikNav.m_rigData, transformHandle, skeletalMeshHandle, m_transformManagerPtr);
			ikNav.m_ikRigController.init();
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
		ComponentManager<IKNavigationComponent>* m_ikNavigationManagerPtr = nullptr;
	};
};


#endif