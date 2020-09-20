#pragma once
#ifndef RIGIDBODYCOMPONENT_HPP
#define RIGIDBODYCOMPONENT_HPP
#include <btBulletDynamicsCommon.h>
#include "PhysicsCollisionSystem.hpp"
#include "CustomMotionState.hpp"
#include "../World/Component.hpp"
#include "../World/ComponentManager.hpp"
#include <memory>
namespace Mona {
	class RigidBodyComponent {
	public:
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "RigidBodyComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::RigidBodyComponent);
		RigidBodyComponent(	InnerComponentHandle transformHandle,
							ComponentManager<TransformComponent>* managerPtr,
							PhysicsCollisionSystem& physicsCollisionSystemPtr,
							const glm::vec3 &halfExtents) 
		{
			m_collisionShapePtr.reset(new btBoxShape(btVector3(halfExtents.x, halfExtents.y, halfExtents.z)));
			btScalar mass(1.0f);
			btVector3 localInertia(0, 0, 0);
			m_collisionShapePtr->calculateLocalInertia(mass, localInertia);
			m_motionStatePtr.reset(new CustomMotionState(transformHandle, managerPtr));
			btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, m_motionStatePtr.get(), m_collisionShapePtr.get(), localInertia);
			m_rigidBodyPtr.reset(new btRigidBody(rbInfo));
			physicsCollisionSystemPtr.AddRigidBody(m_rigidBodyPtr.get());

			
		}
	private:
		std::unique_ptr<CustomMotionState> m_motionStatePtr;
		std::unique_ptr<btCollisionShape> m_collisionShapePtr;
		std::unique_ptr<btRigidBody> m_rigidBodyPtr;


	};
}
#endif