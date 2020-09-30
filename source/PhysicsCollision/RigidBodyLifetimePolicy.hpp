#pragma once
#ifndef RIGIDBODYLIFETIMEPOLICY_HPP
#define RIGIDBODYLIFETIMEPOLICY_HPP
#include "../World/Component.hpp"
#include "../World/ComponentManager.hpp"
#include "PhysicsCollisionSystem.hpp"
#include "RigidBodyComponent.hpp"
namespace Mona {

	class RigidBodyLifetimePolicy {
	public:
		RigidBodyLifetimePolicy() = default;
		RigidBodyLifetimePolicy(ComponentManager<TransformComponent>* managerPtr, PhysicsCollisionSystem* physicsSystemPtr) :
			m_transformManagerPtr(managerPtr), m_physicsCollisionSystemPtr(physicsSystemPtr) {}

		void OnAddComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody) noexcept {
			InnerComponentHandle handle = gameObjectPtr->GetInnerComponentHandle<TransformComponent>();
			rigidBody.InitializeMotionState(handle, m_transformManagerPtr);
			m_physicsCollisionSystemPtr->AddRigidBody(rigidBody);
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody) noexcept {
			m_physicsCollisionSystemPtr->RemoveRigidBody(rigidBody);
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		PhysicsCollisionSystem* m_physicsCollisionSystemPtr = nullptr;
	};
}


#endif