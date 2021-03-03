#pragma once
#ifndef RAYCASTRESULTS_HPP
#define RAYCASTRESULTS_HPP
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <glm/glm.hpp>
#include <vector>
#include "RigidBodyComponent.hpp"
#include "../World/ComponentHandle.hpp"
namespace Mona {
	/*
	* La clase ClosestHitRaycastResult representa el resultado de una consulta de raycast que busca la colisión mas cercana.
	*/
	class ClosestHitRaycastResult {
	public:
		ClosestHitRaycastResult(const btCollisionWorld::ClosestRayResultCallback& btRaycastResult, ComponentManager<RigidBodyComponent>* rigidBodyDatamanager) {
			//El principal trabajo de este constructor es transformar el resultado de una consulta de Bullet a un formato interno
			m_rayFrom = glm::vec3(btRaycastResult.m_rayFromWorld.x(), btRaycastResult.m_rayFromWorld.y(), btRaycastResult.m_rayFromWorld.z());
			m_rayTo = glm::vec3(btRaycastResult.m_rayToWorld.x(), btRaycastResult.m_rayToWorld.y(), btRaycastResult.m_rayToWorld.z());
			m_hitPosition = glm::vec3(btRaycastResult.m_hitPointWorld.x(), btRaycastResult.m_hitPointWorld.y(), btRaycastResult.m_hitPointWorld.z());
			m_hitNormal = glm::vec3(btRaycastResult.m_hitNormalWorld.x(), btRaycastResult.m_hitNormalWorld.y(), btRaycastResult.m_hitNormalWorld.z());
			m_hasHit = btRaycastResult.hasHit();
			if (btRaycastResult.hasHit()) {
				//Bullet permite usar dos indices por cada collisionObject (UserIndex/UserIndex2), con estos
				//podemos recuperar la instancia de RigidBodyComponent asociada al collisionObject de bullet
				InnerComponentHandle rbInnerHandle = InnerComponentHandle(btRaycastResult.m_collisionObject->getUserIndex(), btRaycastResult.m_collisionObject->getUserIndex2());
				m_rigidBody = RigidBodyHandle(rbInnerHandle, rigidBodyDatamanager);
			}
			
		}

		bool HasHit() const {
			return m_hasHit;
		}
		glm::vec3 m_rayFrom;
		glm::vec3 m_rayTo;
		glm::vec3 m_hitPosition;
		glm::vec3 m_hitNormal;
		bool m_hasHit;
		RigidBodyHandle m_rigidBody;
	};

	/*
	* La clase AllHitsRaycastResult representa el resultado de una consulta de raycast que busca todas las colisiones que intersectaron el rayo consultado.
	*/
	class AllHitsRaycastResult {
	public:
		AllHitsRaycastResult(const btCollisionWorld::AllHitsRayResultCallback& btRaycastResult, ComponentManager<RigidBodyComponent>* rigidBodyDatamanager) {
			//El principal trabajo de este constructor es transformar el resultado de una consulta de Bullet a un formato interno
			m_rayFrom = glm::vec3(btRaycastResult.m_rayFromWorld.x(), btRaycastResult.m_rayFromWorld.y(), btRaycastResult.m_rayFromWorld.z());
			m_rayTo = glm::vec3(btRaycastResult.m_rayToWorld.x(), btRaycastResult.m_rayToWorld.y(), btRaycastResult.m_rayToWorld.z());
			auto& hitPositions = btRaycastResult.m_hitPointWorld;
			auto& hitNormals = btRaycastResult.m_hitNormalWorld;
			auto& collisionObjects = btRaycastResult.m_collisionObjects;
			m_hitPositions.reserve(hitPositions.size());
			m_hitNormals.reserve(hitNormals.size());
			m_rigidBodies.reserve(collisionObjects.size());

			for (int i = 0; i < hitPositions.size(); i++)
			{
				const auto& position = hitPositions[i];
				m_hitPositions.emplace_back(position.x(), position.y(), position.z());
			}

			for (int i = 0; i < hitNormals.size(); i++) 
			{
				const auto& normal = hitNormals[i];
				m_hitNormals.emplace_back(normal.x(), normal.y(), normal.z());
			}

			for (int i = 0; i < collisionObjects.size(); i++) {
				//Bullet permite usar dos indices por cada collisionObject (UserIndex/UserIndex2), con estos
				//podemos recuperar la instancia de RigidBodyComponent asociada al collisionObject de bullet
				const auto& collisionObject = collisionObjects[i];
				InnerComponentHandle rbInnerHandle = InnerComponentHandle(btRaycastResult.m_collisionObject->getUserIndex(), btRaycastResult.m_collisionObject->getUserIndex2());
				m_rigidBodies.emplace_back(rbInnerHandle, rigidBodyDatamanager);
			}

		}
		bool HasHit() const {
			return m_rigidBodies.size() > 0;
		}
		glm::vec3 m_rayFrom;
		glm::vec3 m_rayTo;
		std::vector<glm::vec3>  m_hitPositions;
		std::vector<glm::vec3>	m_hitNormals;
		std::vector<RigidBodyHandle> m_rigidBodies;
	};
}
#endif