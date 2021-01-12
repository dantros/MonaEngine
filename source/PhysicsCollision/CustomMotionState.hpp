#pragma once
#ifndef CUSTOMMOTIONSTATE_HPP
#define CUSTOMMOTIONSTATE_HPP
#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>
#include "../World/TransformComponent.hpp"
#include "../World/ComponentManager.hpp"
namespace Mona {
	class CustomMotionState : public btMotionState {
	public:
		CustomMotionState(const glm::vec3& offset = glm::vec3(0.0f)) :
			m_transformHandle(), m_managerPtr(nullptr), m_translationOffset(offset) {}
		virtual void getWorldTransform(btTransform& worldTrans) const override {
			const TransformComponent* transformPtr = m_managerPtr->GetComponentPointer(m_transformHandle);
			const glm::vec3& translation = transformPtr->GetLocalTranslation();
			const glm::fquat& rotation = transformPtr->GetLocalRotation();
			glm::vec3 rotatedTranslation = rotation * m_translationOffset;
			worldTrans.setOrigin(btVector3(translation.x + rotatedTranslation.x,
				translation.y + rotatedTranslation.y,
				translation.z + rotatedTranslation.z));
			worldTrans.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));
		}
		
		virtual void setWorldTransform(const btTransform& worldTrans) override {
			const btQuaternion rotation = worldTrans.getRotation();
			const btVector3& translation = worldTrans.getOrigin();
			glm::fquat rot = glm::fquat(rotation.w(), rotation.x(), rotation.y(), rotation.z());
			glm::vec3 rotatedTrnaslation = rot * m_translationOffset;
			TransformComponent* transformPtr = m_managerPtr->GetComponentPointer(m_transformHandle);
			transformPtr->SetTranslation(glm::vec3(translation.x() - rotatedTrnaslation.x,
				translation.y() - rotatedTrnaslation.y,
				translation.z() - rotatedTrnaslation.z));
			transformPtr->SetRotation(glm::fquat(rotation.w(), rotation.x(), rotation.y(), rotation.z()));

		}

		void Initialize(InnerComponentHandle handle, ComponentManager<TransformComponent>* managerPtr) {
			m_transformHandle = handle;
			m_managerPtr = managerPtr;
		}
		const glm::vec3& GetTranslationOffset() const { return m_translationOffset; }
	private:
		glm::vec3 m_translationOffset;
		InnerComponentHandle m_transformHandle;
		ComponentManager<TransformComponent>* m_managerPtr;
	};

}
#endif