#pragma once
#ifndef IKNAVIGATIONLIFETIMEPOLICY_HPP
#define IKNAVIGATIONLIFETIMEPOLICY_HPP
#include "../World/ComponentManager.hpp"
#include "../World/TransformComponent.hpp"
#include "IKNavigationComponent.hpp"
namespace Mona {
	/*
	* Clase que representa las polizas de agregar y remover una instancia de RigidBodyComponent a un GameObject, es decir,
	* cada vez que se agrega una de esta componente el metodo OnAddComponent es llamado, mientras que cuando se remueve
	* OnRemoveComponent es llamado
	*/
	class IKNavigationLifetimePolicy {
	public:
		IKNavigationLifetimePolicy() = default;

		void OnAddComponent(GameObject* gameObjectPtr, IKNavigationComponent& ikNav, const InnerComponentHandle& handle) noexcept {
			InnerComponentHandle transformHandle = gameObjectPtr->GetInnerComponentHandle<TransformComponent>();
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody, const InnerComponentHandle &handle) noexcept {
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
	};
}


#endif