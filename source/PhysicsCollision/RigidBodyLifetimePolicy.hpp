#pragma once
#ifndef RIGIDBODYLIFETIMEPOLICY_HPP
#define RIGIDBODYLIFETIMEPOLICY_HPP
#include "../World/ComponentManager.hpp"
#include "../World/TransformComponent.hpp"
#include "PhysicsCollisionSystem.hpp"
#include "RigidBodyComponent.hpp"
namespace Mona {
	/*
	* Clase que representa las polizas de agregar y remover una instancia de RigidBodyComponent a un GameObject, es decir,
	* cada vez que se agrega una de esta componente el metodo OnAddComponent es llamado, mientras que cuando se remueve
	* OnRemoveComponent es llamado
	*/
	class RigidBodyLifetimePolicy {
	public:
		RigidBodyLifetimePolicy() = default;
		RigidBodyLifetimePolicy(ComponentManager<TransformComponent>* managerPtr, PhysicsCollisionSystem* physicsSystemPtr) :
			m_transformManagerPtr(managerPtr), m_physicsCollisionSystemPtr(physicsSystemPtr) {}

		void OnAddComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody, const InnerComponentHandle& handle) noexcept {
			InnerComponentHandle transformHandle = gameObjectPtr->GetInnerComponentHandle<TransformComponent>();
			//Se configura la información para poder sincronizar la informacion espacial internal (TransformComponent) 
			//con la de bullet 
			rigidBody.InitializeMotionState(transformHandle, m_transformManagerPtr);
			btRigidBody* rb = rigidBody.m_rigidBodyPtr.get();
			//Es necesario usar UserIndex/UserIndex2 de bullet para a partir de cualquier collisionObject recuperar
			//rapidamente la instancia de RigidBOdyComponent asociada
			rb->setUserIndex(handle.m_index);
			rb->setUserIndex2(handle.m_generation);
			m_physicsCollisionSystemPtr->AddRigidBody(rigidBody);
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody, const InnerComponentHandle &handle) noexcept {
			m_physicsCollisionSystemPtr->RemoveRigidBody(rigidBody);
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		PhysicsCollisionSystem* m_physicsCollisionSystemPtr = nullptr;
	};
}


#endif