#pragma once
#ifndef PHYSICSCOLLISIONSYSTEM_HPP
#define PHYSICSCOLLISIONSYSTEM_HPP
#include <btBulletDynamicsCommon.h>
#include <set>
#include <utility>
#include "RigidBodyComponent.hpp"

namespace Mona {
	class PhysicsCollisionSystem {
	public:
		PhysicsCollisionSystem() {
			m_collisionConfigurationPtr = new btDefaultCollisionConfiguration();
			m_dispatcherPtr = new btCollisionDispatcher(m_collisionConfigurationPtr);
			m_broadphasePtr = new btDbvtBroadphase();
			m_solverPtr = new btSequentialImpulseConstraintSolver();
			m_worldPtr = new btDiscreteDynamicsWorld(m_dispatcherPtr, m_broadphasePtr, m_solverPtr, m_collisionConfigurationPtr);
		}
		~PhysicsCollisionSystem() {
			delete m_worldPtr;
			delete m_solverPtr;
			delete m_broadphasePtr;
			delete m_dispatcherPtr;
			delete m_collisionConfigurationPtr;

		}
		void SetGravity(const glm::vec3& gravity) noexcept;
		glm::vec3 GetGravity() const noexcept;
		void StepSimulation(float timeStep) noexcept;
		void SubmitCollisionEvents(typename RigidBodyComponent::managerType& rigidBodyDatamanager) noexcept;
		void AddRigidBody(RigidBodyComponent& component) noexcept;
		void RemoveRigidBody(RigidBodyComponent& component) noexcept;
		void StartUp(	typename TransformComponent::managerType& transformDataManager,
						typename RigidBodyComponent::managerType& rigidBodyDataManager) noexcept;
		void ShutDown() noexcept;
		btDynamicsWorld* GetPhysicsWorldPtr() noexcept { return m_worldPtr; }
	private:
		btBroadphaseInterface* m_broadphasePtr;
		btCollisionConfiguration* m_collisionConfigurationPtr;
		btCollisionDispatcher* m_dispatcherPtr;
		btConstraintSolver* m_solverPtr;
		btDynamicsWorld* m_worldPtr;

		using CollisionPair = std::pair<const btRigidBody*, const btRigidBody*>;
		using CollisionSet = std::set<CollisionPair>;
		CollisionSet m_previousCollisionSet;
		



	};
}


#endif