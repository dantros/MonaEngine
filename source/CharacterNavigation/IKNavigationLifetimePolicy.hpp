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
	/*
	* Clase que representa las polizas de agregar y remover una instancia de RigidBodyComponent a un GameObject, es decir,
	* cada vez que se agrega una de esta componente el metodo OnAddComponent es llamado, mientras que cuando se remueve
	* OnRemoveComponent es llamado
	*/
	class IKNavigationLifetimePolicy {
	public:
		IKNavigationLifetimePolicy() = default;
		IKNavigationLifetimePolicy(ComponentManager<TransformComponent>* transformManagerPtr, ComponentManager<StaticMeshComponent>* staticMeshManagerPtr, 
			ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr,
			ComponentManager<IKNavigationComponent>* ikNavigationManagerPtr) : 
			m_transformManagerPtr(transformManagerPtr), m_staticMeshManagerPtr(staticMeshManagerPtr), 
			m_skeletalMeshManagerPtr(skeletalMeshManagerPtr), m_rigidBodyManagerPtr(rigidBodyManagerPtr),
		m_ikNavigationManagerPtr(ikNavigationManagerPtr){}

		void OnAddComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
			ikNav.m_skeletalMeshManagerPtr = m_skeletalMeshManagerPtr;
			ikNav.m_staticMeshManagerPtr = m_staticMeshManagerPtr;
			ikNav.m_transformManagerPtr = m_transformManagerPtr;
			ikNav.m_rigidBodyManagerPtr = m_rigidBodyManagerPtr;

			// validar clip base
			InnerComponentHandle skeletalMeshHandle = m_ikNavigationManagerPtr->GetOwner(handle)->GetInnerComponentHandle<SkeletalMeshComponent>();
			std::shared_ptr<Skeleton> skeletonPtr = m_skeletalMeshManagerPtr->GetComponentPointer(skeletalMeshHandle)->GetSkeleton();
			if(ikNav.m_baseAnimationClip->GetSkeleton() != skeletonPtr) {
				MONA_LOG_ERROR("Input animation does not correspond to base skeleton.");
				return;
			}

			InnerComponentHandle rigidBodyHandle = m_ikNavigationManagerPtr->GetOwner(handle)->GetInnerComponentHandle<RigidBodyComponent>();
			ikNav.m_ikRig = IKRig(ikNav.m_baseAnimationClip, ikNav.m_rigData, rigidBodyHandle, skeletalMeshHandle);
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		ComponentManager<StaticMeshComponent>* m_staticMeshManagerPtr = nullptr;
		ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
		ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
		ComponentManager<IKNavigationComponent>* m_ikNavigationManagerPtr = nullptr;
	};
};


#endif