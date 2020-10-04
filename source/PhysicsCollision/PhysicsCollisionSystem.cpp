#include "PhysicsCollisionSystem.hpp"
#include <algorithm>
#include "RigidBodyLifetimePolicy.hpp"
#include <vector>
#include "CollisionInformation.hpp"
#include "../World/ComponentHandle.hpp"
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
	void  PhysicsCollisionSystem::SubmitCollisionEvents(World& world, typename RigidBodyComponent::managerType& rigidBodyDatamanager) noexcept
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
				CollisionPair currentCollisionPair = std::make_tuple(firstSortedBody, secondSortedBody, shouldSwap, i);
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

		std::vector<std::tuple<RigidBodyHandle, RigidBodyHandle, bool, CollisionInformation>> newCollisionsInformation;
		newCollisionsInformation.reserve(newCollisions.size());
		for (auto& newCollision : newCollisions)
		{
			auto& rb0 = get<0>(newCollision);
			auto& rb1 = get<1>(newCollision);
			InnerComponentHandle rbhandle0 = InnerComponentHandle(rb0->getUserIndex(), rb0->getUserIndex2());
			InnerComponentHandle rbHandle1 = InnerComponentHandle(rb1->getUserIndex(), rb1->getUserIndex2());
			newCollisionsInformation.emplace_back(std::make_tuple(	RigidBodyHandle(rbhandle0, &rigidBodyDatamanager),
																	RigidBodyHandle(rbHandle1, &rigidBodyDatamanager),
																	get<2>(newCollision),
																	CollisionInformation(m_dispatcherPtr->getManifoldByIndexInternal(get<3>(newCollision)))));
		}


		for (auto& collisionInformation : newCollisionsInformation) {
			auto& rb0 = get<0>(collisionInformation);
			auto& rb1 = get<1>(collisionInformation);
			auto& collisionInfo = get<3>(collisionInformation);
			if (rb0->HasStartCollisionCallback()) {
				rb0->CallStartCollisionCallback(world, rb1, get<2>(collisionInformation), collisionInfo);
			}
			if (rb1->HasStartCollisionCallback()) {
				rb1->CallStartCollisionCallback(world, rb1, !get<2>(collisionInformation), collisionInfo);
			}
		}

		CollisionSet removedCollisions;
		std::set_difference(m_previousCollisionSet.begin(), m_previousCollisionSet.end(),
							currentCollisionSet.begin(), currentCollisionSet.end(),
							std::inserter(removedCollisions, removedCollisions.begin()));
		std::vector <std::tuple<RigidBodyHandle, RigidBodyHandle>> removedCollisionInformation;

		for (auto& removedCollision : removedCollisions)
		{
			auto& rb0 = get<0>(removedCollision);
			auto& rb1 = get<1>(removedCollision);
			InnerComponentHandle rbhandle0 = InnerComponentHandle(rb0->getUserIndex(), rb0->getUserIndex2());
			InnerComponentHandle rbHandle1 = InnerComponentHandle(rb1->getUserIndex(), rb1->getUserIndex2());
			removedCollisionInformation.emplace_back(std::make_tuple(RigidBodyHandle(rbhandle0, &rigidBodyDatamanager),
				RigidBodyHandle(rbHandle1, &rigidBodyDatamanager)));
		}
		m_previousCollisionSet = currentCollisionSet;

	}
}