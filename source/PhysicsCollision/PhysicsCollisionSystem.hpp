#pragma once
#ifndef PHYSICSCOLLISIONSYSTEM_HPP
#define PHYSICSCOLLISIONSYSTEM_HPP
#include <btBulletDynamicsCommon.h>
#include <set>
#include <tuple>
#include "RigidBodyComponent.hpp"
#include "RaycastResults.hpp"

namespace Mona {
	class World;
	class EventManager;
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
		ClosestHitRaycastResult ClosestHitRayTest(const glm::vec3& rayFrom,
			const glm::vec3& rayTo,
			ComponentManager<RigidBodyComponent>& rigidBodyDatamanager) const;

		AllHitsRaycastResult AllHitsRayTest(const glm::vec3& rayFrom,
			const glm::vec3& rayTo,
			ComponentManager<RigidBodyComponent>& rigidBodyDatamanager) const;

		void StepSimulation(float timeStep) noexcept;
		void SubmitCollisionEvents(World& world,
			EventManager& eventManager,
			ComponentManager<RigidBodyComponent>& rigidBodyDatamanager) noexcept;

		void AddRigidBody(RigidBodyComponent& component) noexcept;
		void RemoveRigidBody(RigidBodyComponent& component) noexcept;
		void ShutDown() noexcept;
		btDynamicsWorld* GetPhysicsWorldPtr() noexcept { return m_worldPtr; }

		using CollisionPair = std::tuple<const btRigidBody*, const btRigidBody*, bool, int>;
		struct cmp {
			bool operator()(const CollisionPair& lhs, const CollisionPair& rhs) const{
				return (bool)(std::get<0>(lhs) < std::get<0>(rhs)) || (!(bool)(std::get<0>(rhs) < std::get<0>(lhs)) &&
					((bool)(std::get<1>(rhs) < std::get<1>(lhs))))
					;
			}
		};
		using CollisionSet = std::set<CollisionPair, cmp>;
	private:
		btBroadphaseInterface* m_broadphasePtr;
		btCollisionConfiguration* m_collisionConfigurationPtr;
		btCollisionDispatcher* m_dispatcherPtr;
		btConstraintSolver* m_solverPtr;
		btDynamicsWorld* m_worldPtr;


		CollisionSet m_previousCollisionSet;
		



	};
}


#endif