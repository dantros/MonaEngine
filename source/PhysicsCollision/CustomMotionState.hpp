#pragma once
#ifndef CUSTOMMOTIONSTATE_HPP
#define CUSTOMMOTIONSTATE_HPP
#include <btBulletDynamicsCommon.h>
#include "../World/TransformComponent.hpp"
#include "../World/ComponentManager.hpp"
namespace Mona {
	class CustomMotionState : public btMotionState {
	public:
		CustomMotionState(InnerComponentHandle handle, ComponentManager<TransformComponent>* managerPtr) :
			m_transformHandle(handle), m_managerPtr(managerPtr) {}
		virtual void getWorldTransform(btTransform& worldTrans) const override {
			const TransformComponent* transformPtr = m_managerPtr->GetComponentPointer(m_transformHandle);
			const glm::vec3& translation = transformPtr->GetLocalTranslation();
			const glm::fquat& rotation = transformPtr->GetLocalRotation();
			worldTrans.setOrigin(btVector3(translation.x, translation.y, translation.z));
			worldTrans.setRotation(btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w));
		}
		
		virtual void setWorldTransform(const btTransform& worldTrans) override {
			const btQuaternion rotation = worldTrans.getRotation();
			const btVector3& translation = worldTrans.getOrigin();
			TransformComponent* transformPtr = m_managerPtr->GetComponentPointer(m_transformHandle);
			transformPtr->SetTranslation(glm::vec3(translation.x(), translation.y(), translation.z()));
			transformPtr->SetRotation(glm::fquat(rotation.w(), rotation.x(), rotation.y(), rotation.z()));

		}
	private:
		InnerComponentHandle m_transformHandle;
		ComponentManager<TransformComponent>* m_managerPtr;
	};

}
#endif