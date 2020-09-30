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


	class RigidBodyLifetimePolicy;

	class RigidBodyComponent {
		friend class PhysicsCollisionSystem;
		friend class RigidBodyLifetimePolicy;
	public:
		using managerType = ComponentManager<RigidBodyComponent, RigidBodyLifetimePolicy>;
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

		void SetLocalScaling(const glm::vec3 &scale) {
			m_collisionShapePtr->setLocalScaling(btVector3(scale.x, scale.y, scale.z));
		}

		glm::vec3 GetLocalScaling() const {
			const btVector3 &scaling = m_collisionShapePtr->getLocalScaling();
			return glm::vec3(scaling.x(), scaling.y(), scaling.z());
		}

		void SetRestitution(float factor) {
			m_rigidBodyPtr->setRestitution(btScalar(factor));
		}

		float GetRestitution() const {
			return m_rigidBodyPtr->getRestitution();
		}

		void SetFriction(float factor) {
			m_rigidBodyPtr->setFriction(btScalar(factor));
		}

		float GetFriction() const {
			m_rigidBodyPtr->getFriction();
		}

		void SetLinearFactor(const glm::vec3& linearFactor) {
			m_rigidBodyPtr->setLinearFactor(btVector3(linearFactor.x, linearFactor.y, linearFactor.z));
		}

		glm::vec3 GetLinearFactor() const {
			const btVector3& linearFactor = m_rigidBodyPtr->getLinearFactor();
			return glm::vec3(linearFactor.x(), linearFactor.y(), linearFactor.z());
		}

		void SetAngularFactor(const glm::vec3& angularFactor){
			m_rigidBodyPtr->setAngularFactor(btVector3(angularFactor.x, angularFactor.y, angularFactor.z));
		}

		glm::vec3 GetAngularFactor() const {
			const btVector3& angularFactor = m_rigidBodyPtr->getAngularFactor();
			return glm::vec3(angularFactor.x(), angularFactor.y(), angularFactor.z());
		}

		void SetLinearVelocity(const glm::vec3& velocity) {
			m_rigidBodyPtr->setLinearVelocity(btVector3(velocity.x, velocity.y, velocity.z));
		}

		glm::vec3 GetLinearVelocity() const {
			const btVector3& linearVelocity = m_rigidBodyPtr->getLinearVelocity();
			return glm::vec3(linearVelocity.x(), linearVelocity.y(), linearVelocity.z());
		}

		void SetAngularVelocity(const glm::vec3& velocity) {
			m_rigidBodyPtr->setAngularVelocity(btVector3(velocity.x, velocity.y, velocity.z));
		}

		glm::vec3 GetAngularVelocity() const {
			const btVector3& angularVelocity = m_rigidBodyPtr->getAngularVelocity();
			return glm::vec3(angularVelocity.x(), angularVelocity.y(), angularVelocity.z());
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
				btRigidBody::btRigidBodyConstructionInfo rbInfo(btMass, nullptr, m_collisionShapePtr.get(), localInertia);
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