#pragma once
#ifndef PHYSICSCOLLISIONSYSTEM_HPP
#define PHYSICSCOLLISIONSYSTEM_HPP
#include <btBulletDynamicsCommon.h>
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
			m_worldPtr->setGravity(btVector3(0.0f, 0.0f, 0.0f));
		}
		~PhysicsCollisionSystem() {
			delete m_worldPtr;
			delete m_solverPtr;
			delete m_broadphasePtr;
			delete m_dispatcherPtr;
			delete m_collisionConfigurationPtr;

		}
		void StepSimulation(float timeStep) noexcept;
		void AddRigidBody(RigidBodyComponent& component) noexcept;
		void RemoveRigidBody(RigidBodyComponent& component) noexcept;
		void ShutDown() noexcept;
		btDynamicsWorld* GetPhysicsWorldPtr() noexcept { return m_worldPtr; }
	private:
		btBroadphaseInterface* m_broadphasePtr;
		btCollisionConfiguration* m_collisionConfigurationPtr;
		btCollisionDispatcher* m_dispatcherPtr;
		btConstraintSolver* m_solverPtr;
		btDynamicsWorld* m_worldPtr;

	};
}


#endif