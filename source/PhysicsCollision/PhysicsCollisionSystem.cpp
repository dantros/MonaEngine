#include "PhysicsCollisionSystem.hpp"
#include <algorithm>
#include "RigidBodyLifetimePolicy.hpp"
namespace Mona {
	void PhysicsCollisionSystem::StepSimulation(float timeStep) noexcept {
		m_worldPtr->stepSimulation(timeStep);	
	}

	void PhysicsCollisionSystem::AddRigidBody(RigidBodyComponent &rigidBody) noexcept {
		m_worldPtr->addRigidBody(rigidBody.m_rigidBodyPtr.get());
	}

	void PhysicsCollisionSystem::RemoveRigidBody(RigidBodyComponent& rigidBody) noexcept {
		m_worldPtr->removeRigidBody(rigidBody.m_rigidBodyPtr.get());
	}

	void PhysicsCollisionSystem::StartUp(typename TransformComponent::managerType& transformDataManager, typename RigidBodyComponent::managerType& rigidBodyDataManager) noexcept
	{
		rigidBodyDataManager.SetLifetimePolicy(RigidBodyLifetimePolicy(&transformDataManager, this));
	}

	void PhysicsCollisionSystem::ShutDown() noexcept {
		for (int i = 0; i < m_worldPtr->getNumCollisionObjects(); i++) {
			m_worldPtr->removeCollisionObject(m_worldPtr->getCollisionObjectArray()[i]);
		}
	}

	void PhysicsCollisionSystem::SetGravity(const glm::vec3& gravity) noexcept {
		m_worldPtr->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
	}

	glm::vec3 PhysicsCollisionSystem::GetGravity() const noexcept {
		const btVector3& gravity = m_worldPtr->getGravity();
		return glm::vec3(gravity.x(), gravity.y(), gravity.z());
	}
	void  PhysicsCollisionSystem::SubmitCollisionEvents(typename RigidBodyComponent::managerType& rigidBodyDatamanager) noexcept
	{
		CollisionSet currentCollisionSet;
		auto manifoldNum = m_dispatcherPtr->getNumManifolds();
		for (decltype(manifoldNum) i = 0; i < manifoldNum; i++) {
			btPersistentManifold* manifoldPtr = m_dispatcherPtr->getManifoldByIndexInternal(i);
			auto numContacts = manifoldPtr->getNumContacts();
			if (numContacts > 0) {
				const btRigidBody* body0 = static_cast<const btRigidBody*>(manifoldPtr->getBody0());
				const btRigidBody* body1 = static_cast<const btRigidBody*>(manifoldPtr->getBody1());
				const bool shouldSwap = body0 > body1;
				const btRigidBody* firstSortedBody = shouldSwap ? body1 : body0;
				const btRigidBody* secondSortedBody = shouldSwap ? body0 : body1;
				CollisionPair currentCollisionPair = std::make_pair(firstSortedBody, secondSortedBody);
				currentCollisionSet.insert(currentCollisionPair);
				/*
				if (m_previousCollisionSet.find(currentCollisionPair) == m_previousCollisionSet.end()) {
					//MONA_LOG_INFO("NEW COLLISION");
					
					InnerComponentHandle rbhandle0 = InnerComponentHandle(firstSortedBody->getUserIndex(), firstSortedBody->getUserIndex2());
					InnerComponentHandle rbHandle1 = InnerComponentHandle(secondSortedBody->getUserIndex(), secondSortedBody->getUserIndex2());
					RigidBodyComponent* myRigidBody0 = rigidBodyDatamanager.GetComponentPointer(rbhandle0);
					RigidBodyComponent* myRigidBody1 = rigidBodyDatamanager.GetComponentPointer(rbHandle1);
					for (int i = 0; i < numContacts; i++)
					{
						const btManifoldPoint& point = manifoldPtr->getContactPoint(i);
						const btVector3& normal = point.m_normalWorldOnB;
						//MONA_LOG_INFO("Normal first object =  ({0},{1},{2})", normal.x(), normal.y(), normal.z());
					}
				}*/
			}
		}

		CollisionSet newCollisions;
		std::set_difference(currentCollisionSet.begin(), currentCollisionSet.end(),
							m_previousCollisionSet.begin(), m_previousCollisionSet.end(),
							std::inserter(newCollisions, newCollisions.begin()));
		for (auto& newCollision : newCollisions)
		{
			MONA_LOG_INFO("COLLISION Started");
		}
		CollisionSet removedCollisions;
		std::set_difference(m_previousCollisionSet.begin(), m_previousCollisionSet.end(),
							currentCollisionSet.begin(), currentCollisionSet.end(),
							std::inserter(removedCollisions, removedCollisions.begin()));
		for (auto& removedCollision : removedCollisions)
		{
			MONA_LOG_INFO("COLLISION ENDED");
		}
		m_previousCollisionSet = currentCollisionSet;

	}
}