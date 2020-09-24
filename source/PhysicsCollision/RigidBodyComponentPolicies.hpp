#pragma once
#ifndef RIGIDBODYCOMPONENTPOLICIES_HPP
#define RIGIDBODYCOMPONENTPOLICIES_HPP
#include "../World/Component.hpp"
#include "../World/ComponentManager.hpp"
#include "PhysicsCollisionSystem.hpp"
#include "RigidBodyComponent.hpp"
namespace Mona {
	class RigidBodyAddPolicy {
	public:
		RigidBodyAddPolicy() = default;
		RigidBodyAddPolicy(ComponentManager<TransformComponent>* managerPtr, PhysicsCollisionSystem* physicsSystemPtr) :
			m_transformManagerPtr(managerPtr), m_physicsCollisionSystemPtr(physicsSystemPtr) {}
		void Apply(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody) noexcept {
			InnerComponentHandle handle = gameObjectPtr->GetInnerComponentHandle<TransformComponent>();
			rigidBody.InitializeMotionState(handle, m_transformManagerPtr);
			m_physicsCollisionSystemPtr->AddRigidBody(rigidBody);
		}
	private:
		ComponentManager<TransformComponent>* m_transformManagerPtr = nullptr;
		PhysicsCollisionSystem* m_physicsCollisionSystemPtr = nullptr;

	};

	class RigidBodyRemovePolicy {
	public:
		RigidBodyRemovePolicy() = default;
		RigidBodyRemovePolicy(PhysicsCollisionSystem* physicsSystemPtr) :
			m_physicsCollisionSystemPtr(physicsSystemPtr) {}
		void Apply(RigidBodyComponent& rigidBody) noexcept {
			m_physicsCollisionSystemPtr->RemoveRigidBody(rigidBody);
		}
	private:
		PhysicsCollisionSystem* m_physicsCollisionSystemPtr = nullptr;
	};
}


#endif