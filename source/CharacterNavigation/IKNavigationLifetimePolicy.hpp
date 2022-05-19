#pragma once
#ifndef IKNAVIGATIONLIFETIMEPOLICY_HPP
#define IKNAVIGATIONLIFETIMEPOLICY_HPP
#include "../World/ComponentManager.hpp"
#include "../World/TransformComponent.hpp"
#include "../PhysicsCollision/RigidBodyComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "IKNavigationComponent.hpp"
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
			ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr) : 
			m_transformManagerPtr(transformManagerPtr), m_staticMeshManagerPtr(staticMeshManagerPtr), 
			m_skeletalMeshManagerPtr(skeletalMeshManagerPtr), m_rigidBodyManagerPtr(rigidBodyManagerPtr) {

		}

		void OnAddComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
			ikNav.m_skeletalMeshManagerPtr = m_skeletalMeshManagerPtr;
			ikNav.m_staticMeshManagerPtr = m_staticMeshManagerPtr;
			ikNav.m_transformManagerPtr = m_transformManagerPtr;
			ikNav.m_rigidBodyManagerPtr = m_rigidBodyManagerPtr;
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		ComponentManager<StaticMeshComponent>* m_staticMeshManagerPtr = nullptr;
		ComponentManager<SkeletalMeshComponent>* m_skeletalMeshManagerPtr = nullptr;
		ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
	};
};


#endif