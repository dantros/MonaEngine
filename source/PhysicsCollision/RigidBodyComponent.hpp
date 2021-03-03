#pragma once
#ifndef RIGIDBODYCOMPONENT_HPP
#define RIGIDBODYCOMPONENT_HPP
#include <btBulletDynamicsCommon.h>
#include "CustomMotionState.hpp"
#include "ShapeTypes.hpp"
#include "CollisionInformation.hpp"
#include "../World/ComponentHandle.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "../Core/Log.hpp"
#include <memory>
#include <functional>
namespace Mona {

	enum class RigidBodyType {
		StaticBody,
		DynamicBody,
		KinematicBody
	};


	class RigidBodyLifetimePolicy;
	class TransformComponent;
	class World;
	class RigidBodyComponent {
		friend class PhysicsCollisionSystem;
		friend class RigidBodyLifetimePolicy;
	public:
		//using managerType = ComponentManager<RigidBodyComponent, RigidBodyLifetimePolicy>;
		using LifetimePolicyType = RigidBodyLifetimePolicy;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "RigidBodyComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::RigidBodyComponent);

		using StartCollisionCallback = std::function<void(World&, RigidBodyHandle&, bool, CollisionInformation&)>;
		using EndCollisionCallback = std::function<void(World&, RigidBodyHandle&)>;
		RigidBodyComponent(const BoxShapeInformation& boxInformation,
			RigidBodyType rigidBodyType,
			float mass = 1.0f,
			bool isTrigger = false,
			const glm::vec3& offset = glm::vec3(0.0f))
		{
			const glm::vec3& halfExtents = boxInformation.m_boxHalfExtents;
			m_collisionShapePtr.reset(new btBoxShape(btVector3(halfExtents.x, halfExtents.y, halfExtents.z)));
			InitializeRigidBody(mass, rigidBodyType, isTrigger);
			m_motionStatePtr.reset(new CustomMotionState(offset));

		}
		RigidBodyComponent(const ConeShapeInformation& coneInformation,
			RigidBodyType rigidBodyType,
			float mass = 1.0f,
			bool isTrigger = false,
			const glm::vec3& offset = glm::vec3(0.0f))
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
			InitializeRigidBody(mass, rigidBodyType, isTrigger);
			m_motionStatePtr.reset(new CustomMotionState(offset));
		}

		RigidBodyComponent(const SphereShapeInformation& sphereInformation,
			RigidBodyType rigidBodyType,
			float mass = 1.0f,
			bool isTrigger = false,
			const glm::vec3& offset = glm::vec3(0.0f))
		{
			
			m_collisionShapePtr.reset(new btSphereShape(sphereInformation.m_radius));
			InitializeRigidBody(mass, rigidBodyType, isTrigger);
			m_motionStatePtr.reset(new CustomMotionState(offset));
		}

		RigidBodyComponent(const CapsuleShapeInformation& capsuleInformation,
			RigidBodyType rigidBodyType,
			float mass = 1.0f,
			bool isTrigger = false,
			const glm::vec3& offset = glm::vec3(0.0f))
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
			InitializeRigidBody(mass, rigidBodyType, isTrigger);
			m_motionStatePtr.reset(new CustomMotionState(offset));
		}

		RigidBodyComponent(const CylinderShapeInformation& cylinderInformation,
			RigidBodyType rigidBodyType,
			float mass = 1.0f,
			bool isTrigger = false,
			const glm::vec3& offset = glm::vec3(0.0f))
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
			InitializeRigidBody(mass, rigidBodyType, isTrigger);
			m_motionStatePtr.reset(new CustomMotionState(offset));
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
			return m_rigidBodyPtr->getFriction();
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
			m_rigidBodyPtr->setActivationState(ACTIVE_TAG);
			m_rigidBodyPtr->setLinearVelocity(btVector3(velocity.x, velocity.y, velocity.z));
		}

		glm::vec3 GetLinearVelocity() const {
			const btVector3& linearVelocity = m_rigidBodyPtr->getLinearVelocity();
			return glm::vec3(linearVelocity.x(), linearVelocity.y(), linearVelocity.z());
		}

		void SetAngularVelocity(const glm::vec3& velocity) {
			m_rigidBodyPtr->setActivationState(ACTIVE_TAG);
			m_rigidBodyPtr->setAngularVelocity(btVector3(velocity.x, velocity.y, velocity.z));
		}

		glm::vec3 GetAngularVelocity() const {
			const btVector3& angularVelocity = m_rigidBodyPtr->getAngularVelocity();
			return glm::vec3(angularVelocity.x(), angularVelocity.y(), angularVelocity.z());
		}

		void SetDamping(float linearDamping, float angularDamping) {
			m_rigidBodyPtr->setDamping(btScalar(linearDamping), btScalar(angularDamping));
		}
		float GetLinearDamping() const {
			return m_rigidBodyPtr->getLinearDamping();
		}
		float GetAngularDamping() const {
			return m_rigidBodyPtr->getAngularDamping();
		}
		void ApplyForce(const glm::vec3& force, const glm::vec3& rel_pos = glm::vec3(0.0f)) {
			m_rigidBodyPtr->setActivationState(ACTIVE_TAG);
			const btVector3& bulletForce = btVector3(force.x, force.y, force.z);
			const btVector3& bulletPos = btVector3(rel_pos.x, rel_pos.y, rel_pos.z);
			m_rigidBodyPtr->applyForce(bulletForce, bulletPos);
		}

		void ApplyTorque(const glm::vec3& torque) {
			m_rigidBodyPtr->setActivationState(ACTIVE_TAG);
			const btVector3& bulletTorque = btVector3(torque.x, torque.y, torque.z);
			m_rigidBodyPtr->applyTorque(bulletTorque);
		}

		bool IsTrigger() const {
			return m_rigidBodyPtr->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE;
		}

		void ClearForces() {
			m_rigidBodyPtr->clearForces();
		}
		glm::vec3 GetTotalForce() const {
			const btVector3& totalForce = m_rigidBodyPtr->getTotalForce();
			return glm::vec3(totalForce.x(), totalForce.y(), totalForce.z());
		}
		void SetStartCollisionCallback(StartCollisionCallback callback) {
			m_onStartCollisionCallback = callback;
		}
		void SetEndCollisionCallback(EndCollisionCallback callback) {
			m_onEndCollisionCallback = callback;
		}

		template <typename ObjectType>
		void SetStartCollisionCallback(ObjectType* obj, void (ObjectType::* memberFunction)(World&, RigidBodyHandle&, bool, CollisionInformation&))
		{
			auto callback = [obj, memberFunction](World& w, RigidBodyHandle& rb, bool b, CollisionInformation&ci) { (obj->*memberFunction)(w, rb, b, ci); };
			m_onStartCollisionCallback = callback;
		}

		template <typename ObjectType>
		void SetEndCollisionCallback(ObjectType* obj, void (ObjectType::* memberFunction)(World&, RigidBodyHandle&)) {
			auto callback = [obj, memberFunction](World& w, RigidBodyHandle& rb) { (obj->*memberFunction)(w, rb); };
			m_onEndCollisionCallback = callback;

		}
		bool HasStartCollisionCallback() const {
			return (bool) m_onStartCollisionCallback;
		}

		bool HasEndCollisionCallback() const {
			return (bool) m_onEndCollisionCallback;
		}

		void CallStartCollisionCallback(World& world, RigidBodyHandle& rb, bool isSwaped, CollisionInformation& information) {
			m_onStartCollisionCallback(world, rb, isSwaped, information);
		}

		void CallEndCollisionCallback(World& world, RigidBodyHandle& rb) {
			m_onEndCollisionCallback(world, rb);
		}

		const glm::vec3& GetTranslationOffset() const { return m_motionStatePtr->GetTranslationOffset(); }

	private:
		void InitializeRigidBody(float mass, RigidBodyType rigidBodyType, bool isTrigger)
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
				//m_rigidBodyPtr->setActivationState(DISABLE_DEACTIVATION);
				
			}

			if(isTrigger){
				int collisionFlags = m_rigidBodyPtr->getCollisionFlags();
				collisionFlags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;
				m_rigidBodyPtr->setCollisionFlags(collisionFlags);
			}
		}

		void InitializeMotionState(InnerComponentHandle transformHandle, ComponentManager<TransformComponent>* managerPtr) {
			m_motionStatePtr->Initialize(transformHandle, managerPtr);
			m_rigidBodyPtr->setMotionState(m_motionStatePtr.get());
		}
		std::unique_ptr<CustomMotionState> m_motionStatePtr;
		std::unique_ptr<btCollisionShape> m_collisionShapePtr;
		std::unique_ptr<btRigidBody> m_rigidBodyPtr;
		StartCollisionCallback m_onStartCollisionCallback;
		EndCollisionCallback m_onEndCollisionCallback;


	};

	
}
#endif