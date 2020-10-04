#pragma once
#ifndef COLLISIONINFORMATION_HPP
#define COLLISIONINFORMATION_HPP
#include <vector>
#include <glm/glm.hpp>
#include <BulletCollision/NarrowPhaseCollision/btPersistentManifold.h>
namespace Mona {
	struct ContactPoint {
	public:
		ContactPoint(const btVector3& positionOnA, const btVector3& positionOnB, const btVector3& normalOnB) :
		positionOnFirstBody(positionOnA.x(), positionOnA.y(), positionOnA.z()),
		positionOnSecondBody(positionOnB.x(), positionOnB.y(), positionOnB.z()),
		normalOnSecondBody(normalOnB.x(), normalOnB.y(), normalOnB.z())
		{}
		glm::vec3 positionOnFirstBody;
		glm::vec3 positionOnSecondBody;
		glm::vec3 normalOnSecondBody;
	};
	class CollisionInformation {
	public:
		using size_type = std::vector<ContactPoint>::size_type;
		CollisionInformation(const btPersistentManifold* manifold) {
			auto numContacts = manifold->getNumContacts();
			m_contactPoints.reserve(numContacts);
			for (int i = 0; i < numContacts; i++) {
				const btManifoldPoint& cp = manifold->getContactPoint(i);
				m_contactPoints.emplace_back(cp.getPositionWorldOnA(), cp.getPositionWorldOnB(), cp.m_normalWorldOnB);
			}
		}
		const ContactPoint& GetCollisionPoint(size_type index) const {
			return m_contactPoints[index];
		}
		size_type GetNumContactPoints() const {
			return m_contactPoints.size();
		}
		
	private:
		std::vector<ContactPoint> m_contactPoints;
	};
}
#endif