#pragma once
#ifndef RIGIDBODYLIFETIMEPOLICY_HPP
#define RIGIDBODYLIFETIMEPOLICY_HPP
#include "../World/ComponentManager.hpp"
#include "../World/TransformComponent.hpp"
#include "PhysicsCollisionSystem.hpp"
#include "RigidBodyComponent.hpp"
namespace Mona {

	class RigidBodyLifetimePolicy {
	public:
		RigidBodyLifetimePolicy() = default;
		RigidBodyLifetimePolicy(typename TransformComponent::managerType* managerPtr, PhysicsCollisionSystem* physicsSystemPtr) :
			m_transformManagerPtr(managerPtr), m_physicsCollisionSystemPtr(physicsSystemPtr) {}

		void OnAddComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody, const InnerComponentHandle& handle) noexcept {
			InnerComponentHandle transformHandle = gameObjectPtr->GetInnerComponentHandle<TransformComponent>();
			rigidBody.InitializeMotionState(transformHandle, m_transformManagerPtr);
			btRigidBody* rb = rigidBody.m_rigidBodyPtr.get();
			rb->setUserIndex(handle.m_index);
			rb->setUserIndex2(handle.m_generation);
			m_physicsCollisionSystemPtr->AddRigidBody(rigidBody);
		}
		void OnRemoveComponent(GameObject* gameObjectPtr, RigidBodyComponent& rigidBody, const InnerComponentHandle &handle) noexcept {
			m_physicsCollisionSystemPtr->RemoveRigidBody(rigidBody);
		}
	private:
		typename TransformComponent::managerType* m_transformManagerPtr = nullptr;
		PhysicsCollisionSystem* m_physicsCollisionSystemPtr = nullptr;
	};
}


#endif