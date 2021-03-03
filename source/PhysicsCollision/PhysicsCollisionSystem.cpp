#include "PhysicsCollisionSystem.hpp"
#include "RigidBodyLifetimePolicy.hpp"
#include <algorithm>
#include <vector>
#include "CollisionInformation.hpp"
#include "../PhysicsCollision/PhysicsCollisionEvents.hpp"
#include "../World/ComponentHandle.hpp"
#include "../Event/EventManager.hpp"
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
	void  PhysicsCollisionSystem::SubmitCollisionEvents(World& world,
		EventManager& eventManager,
		ComponentManager<RigidBodyComponent>& rigidBodyDatamanager) noexcept
	{
		CollisionSet currentCollisionSet;
		
		auto manifoldNum = m_dispatcherPtr->getNumManifolds();
		//Primero se pobla currentCollisionSet con las collisiones en esta iteración
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
			}
		}

		CollisionSet newCollisions;
		//Para encontrar las colisiones nuevas es necesario encontrar las colisiones que estan presentes en la iteración actual
		//pero no en la anterior.
		std::set_difference(currentCollisionSet.begin(), currentCollisionSet.end(),
							m_previousCollisionSet.begin(), m_previousCollisionSet.end(),
							std::inserter(newCollisions, newCollisions.begin()));

		
		std::vector<std::tuple<RigidBodyHandle, RigidBodyHandle, bool, CollisionInformation>> newCollisionsInformation;
		//A partir del conjunto de colisiones nuevas poblado con byRigidBody* se genera un conjunto 
		// con una representación interna RigidBodyHandle.
		newCollisionsInformation.reserve(newCollisions.size());
		for (auto& newCollision : newCollisions)
		{
			auto& rb0 = std::get<0>(newCollision);
			auto& rb1 = std::get<1>(newCollision);
			InnerComponentHandle rbhandle0 = InnerComponentHandle(rb0->getUserIndex(), rb0->getUserIndex2());
			InnerComponentHandle rbHandle1 = InnerComponentHandle(rb1->getUserIndex(), rb1->getUserIndex2());
			newCollisionsInformation.emplace_back(std::make_tuple(	RigidBodyHandle(rbhandle0, &rigidBodyDatamanager),
																	RigidBodyHandle(rbHandle1, &rigidBodyDatamanager),
																	std::get<2>(newCollision),
																	CollisionInformation(m_dispatcherPtr->getManifoldByIndexInternal(std::get<3>(newCollision)))));
		}

		//Se procede a llamar las callbacks correspondientes de cada RigidBodyComponent entrando en una nueva colision.
		//Este proceso podría hacerse en el paso anterior, pero dado que callbacks podrían eliminar componentes en uso en ese momento
		//es necesario posponer este proceso.
		for (auto& collisionInformation : newCollisionsInformation) {
			auto& rb0 = std::get<0>(collisionInformation);
			auto& rb1 = std::get<1>(collisionInformation);
			auto& collisionInfo = std::get<3>(collisionInformation);
			if (rb0.IsValid() && rb0->HasStartCollisionCallback()) {
				rb0->CallStartCollisionCallback(world, rb1, std::get<2>(collisionInformation), collisionInfo);
			}
			if (rb1.IsValid() && rb1->HasStartCollisionCallback()) {
				rb1->CallStartCollisionCallback(world, rb1, !std::get<2>(collisionInformation), collisionInfo);
			}
			//Ademas se publica el evento de colision a travez del eventManager
			eventManager.Publish(StartCollisionEvent(rb0,rb1, std::get<2>(collisionInformation), collisionInfo));
		}

		//El mismo proceso es necesario para colisiones que estan terminando.
		CollisionSet removedCollisions;
		std::set_difference(m_previousCollisionSet.begin(), m_previousCollisionSet.end(),
							currentCollisionSet.begin(), currentCollisionSet.end(),
							std::inserter(removedCollisions, removedCollisions.begin()));
		std::vector <std::tuple<RigidBodyHandle, RigidBodyHandle>> removedCollisionInformation;

		for (auto& removedCollision : removedCollisions)
		{
			auto& rb0 = std::get<0>(removedCollision);
			auto& rb1 = std::get<1>(removedCollision);
			InnerComponentHandle rbhandle0 = InnerComponentHandle(rb0->getUserIndex(), rb0->getUserIndex2());
			InnerComponentHandle rbHandle1 = InnerComponentHandle(rb1->getUserIndex(), rb1->getUserIndex2());
			removedCollisionInformation.emplace_back(std::make_tuple(RigidBodyHandle(rbhandle0, &rigidBodyDatamanager),
				RigidBodyHandle(rbHandle1, &rigidBodyDatamanager)));
		}

		for (auto& collisionInformation : removedCollisionInformation) {
			auto& rb0 = std::get<0>(collisionInformation);
			auto& rb1 = std::get<1>(collisionInformation);
			if (rb0.IsValid() && rb0->HasEndCollisionCallback()) {
				rb0->CallEndCollisionCallback(world, rb1);
			}
			if (rb1.IsValid() && rb1->HasEndCollisionCallback()) {
				rb1->CallEndCollisionCallback(world, rb0);
			}
			eventManager.Publish(EndCollisionEvent(rb0,rb1));
		}
		m_previousCollisionSet = currentCollisionSet;

	}

	ClosestHitRaycastResult PhysicsCollisionSystem::ClosestHitRayTest(const glm::vec3& rayFrom,
		const glm::vec3& rayTo,
		ComponentManager<RigidBodyComponent>& rigidBodyDatamanager) const {

		const btVector3 btFrom = btVector3(rayFrom.x, rayFrom.y, rayFrom.z);
		const btVector3 btTo = btVector3(rayTo.x, rayTo.y, rayTo.z);
		btCollisionWorld::ClosestRayResultCallback rayTest(btFrom, btTo);
		m_worldPtr->rayTest(btFrom, btTo, rayTest);
		return ClosestHitRaycastResult(rayTest, &rigidBodyDatamanager);

	}

	AllHitsRaycastResult PhysicsCollisionSystem::AllHitsRayTest(const glm::vec3& rayFrom,
		const glm::vec3& rayTo,
		ComponentManager<RigidBodyComponent>& rigidBodyDatamanager) const {

		const btVector3 btFrom = btVector3(rayFrom.x, rayFrom.y, rayFrom.z);
		const btVector3 btTo = btVector3(rayTo.x, rayTo.y, rayTo.z);
		btCollisionWorld::AllHitsRayResultCallback rayTest(btFrom, btTo);
		m_worldPtr->rayTest(btFrom, btTo, rayTest);
		return AllHitsRaycastResult(rayTest, &rigidBodyDatamanager);
	}
}