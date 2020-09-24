#pragma once
#ifndef RIGIDBODYCOMPONENT_HPP
#define RIGIDBODYCOMPONENT_HPP
#include <btBulletDynamicsCommon.h>
#include "CustomMotionState.hpp"
#include "ShapeTypes.hpp"
#include "../World/Component.hpp"
#include "../World/ComponentManager.hpp"
#include "../Core/Log.hpp"
#include <memory>
namespace Mona {

	enum class RigidBodyType {
		StaticBody,
		DynamicBody,
		KinematicBody
	};


	class RigidBodyRemovePolicy;
	class RigidBodyAddPolicy;
	class RigidBodyComponent {
		friend class PhysicsCollisionSystem;
		friend class RigidBodyAddPolicy;
	public:
		using managerType = ComponentManager<RigidBodyComponent, RigidBodyAddPolicy, RigidBodyRemovePolicy>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "RigidBodyComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::RigidBodyComponent);
		RigidBodyComponent(	const BoxShapeInformation& boxInformation,
							RigidBodyType rigidBodyType,
							float mass = 1.0f)
		{
			const glm::vec3& halfExtents = boxInformation.m_boxHalfExtents;
			m_collisionShapePtr.reset(new btBoxShape(btVector3(halfExtents.x, halfExtents.y, halfExtents.z)));
			InitializeRigidBody(mass, rigidBodyType);

		}
		RigidBodyComponent(	const ConeShapeInformation& coneInformation,
							RigidBodyType rigidBodyType,
							float mass = 1.0f)
		{
			switch (coneInformation.m_alignment) {
				case(ShapeAlignment::X) : {
					m_collisionShapePtr.reset(new btConeShapeX(coneInformation.m_radius, coneInformation.m_height));
					break;
				}
				case(ShapeAlignment::Y): {
					m_collisionShapePtr.reset(new btConeShape(coneInformation.m_radius, coneInformation.m_height));
					break;
				}
				case(ShapeAlignment::Z): {
					m_collisionShapePtr.reset(new btConeShapeZ(coneInformation.m_radius, coneInformation.m_height));
					break;
				}
			}
			InitializeRigidBody(mass, rigidBodyType);
		}

		RigidBodyComponent(	const SphereShapeInformation& sphereInformation,
							RigidBodyType rigidBodyType,
							float mass = 1.0f) {
			
			m_collisionShapePtr.reset(new btSphereShape(sphereInformation.m_radius));
			InitializeRigidBody(mass, rigidBodyType);

		}

		RigidBodyComponent(	const CapsuleShapeInformation& capsuleInformation,
							RigidBodyType rigidBodyType,
							float mass = 1.0f)
		{
			switch (capsuleInformation.m_alignment) {
			case(ShapeAlignment::X): {
				m_collisionShapePtr.reset(new btCapsuleShapeX(capsuleInformation.m_radius, capsuleInformation.m_height));
				break;
			}
			case(ShapeAlignment::Y): {
				m_collisionShapePtr.reset(new btCapsuleShape(capsuleInformation.m_radius, capsuleInformation.m_height));
				break;
			}
			case(ShapeAlignment::Z): {
				m_collisionShapePtr.reset(new btCapsuleShapeZ(capsuleInformation.m_radius, capsuleInformation.m_height));
				break;
			}
			}
			InitializeRigidBody(mass, rigidBodyType);
		}

		RigidBodyComponent(	const CylinderShapeInformation& cylinderInformation,
							RigidBodyType rigidBodyType,
							float mass = 1.0f)
		{
			const glm::vec3& halfExtents = cylinderInformation.m_cylinderHalfExtents;
			const btVector3 btExtents(halfExtents.x, halfExtents.y, halfExtents.z);
			switch (cylinderInformation.m_alignment) {
			case(ShapeAlignment::X): {
				m_collisionShapePtr.reset(new btCylinderShapeX(btExtents));
				break;
			}
			case(ShapeAlignment::Y): {
				m_collisionShapePtr.reset(new btCylinderShape(btExtents));
				break;
			}
			case(ShapeAlignment::Z): {
				m_collisionShapePtr.reset(new btCylinderShapeZ(btExtents));
				break;
			}
			}
			InitializeRigidBody(mass, rigidBodyType);
		}


	private:
		void InitializeRigidBody(float mass, RigidBodyType rigidBodyType)
		{
			if (rigidBodyType == RigidBodyType::StaticBody || 
				rigidBodyType == RigidBodyType::KinematicBody ||
				mass == 0.0f) {
				btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(0.0f), nullptr, m_collisionShapePtr.get());
				m_rigidBodyPtr.reset(new btRigidBody(rbInfo));
				if (rigidBodyType == RigidBodyType::KinematicBody)
				{
					m_rigidBodyPtr->setCollisionFlags(m_rigidBodyPtr->getCollisionFlags() |
						btCollisionObject::CF_KINEMATIC_OBJECT);
					m_rigidBodyPtr->setActivationState(DISABLE_DEACTIVATION);
				}
			}
			else {
				btScalar btMass(mass);
				btVector3 localInertia(0, 0, 0);
				m_collisionShapePtr->calculateLocalInertia(btMass, localInertia);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(0.0f), nullptr, m_collisionShapePtr.get(), localInertia);
				m_rigidBodyPtr.reset(new btRigidBody(rbInfo));
			}
			
			
		}

		void InitializeMotionState(InnerComponentHandle transformHandle, ComponentManager<TransformComponent>* managerPtr) {
			m_motionStatePtr.reset(new CustomMotionState(transformHandle, managerPtr));
			m_rigidBodyPtr->setMotionState(m_motionStatePtr.get());
		}
		std::unique_ptr<CustomMotionState> m_motionStatePtr;
		std::unique_ptr<btCollisionShape> m_collisionShapePtr;
		std::unique_ptr<btRigidBody> m_rigidBodyPtr;


	};
}
#endif