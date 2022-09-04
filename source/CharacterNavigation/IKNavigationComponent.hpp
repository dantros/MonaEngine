#pragma once
#ifndef IKNAVIGATIONCOMPONENT_HPP
#define IKNAVIGATIONCOMPONENT_HPP

#include <string_view>
#include "../World/ComponentTypes.hpp"
#include "IKRigController.hpp"
namespace Mona {
	class IKNavigationLifetimePolicy;
	class RigData;
	class IKNavigationComponent{
		public:
			friend class IKNavigationLifetimePolicy;
			using LifetimePolicyType = IKNavigationLifetimePolicy;
			using dependencies = DependencyList<SkeletalMeshComponent>;
			static constexpr std::string_view componentName = "IKNavigationComponent";
			static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::IKNavigationComponent);
			IKNavigationComponent(RigData& rigData) {
				m_rigData = rigData;
			}
			void AddAnimation(std::shared_ptr<AnimationClip> animationClip, AnimationType animationType = AnimationType::WALKING) {
				m_ikRigController.addAnimation(animationClip, animationType);
			}

			void SetStrideValidation(bool validateStrides) {
				m_ikRigController.m_ikRig.m_trajectoryGenerator.enableStrideValidation(validateStrides);
			}

			void SetStrideCorrection(bool correctStrides) {
				m_ikRigController.m_ikRig.m_trajectoryGenerator.enableStrideCorrection(correctStrides);
			}

			void EnableIK(bool enableIK) {
				m_ikRigController.enableIK(enableIK);
			}

			int RemoveAnimation(std::shared_ptr<AnimationClip> animationClip) {
				return m_ikRigController.removeAnimation(animationClip);
			}
			void AddTerrain(const GameObjectHandle<GameObject>& staticMeshObject) {
				m_ikRigController.m_ikRig.m_trajectoryGenerator.m_environmentData.addTerrain(staticMeshObject);
			}
			int RemoveTerrain(const GameObjectHandle<GameObject>& staticMeshObject) {
				return m_ikRigController.m_ikRig.m_trajectoryGenerator.m_environmentData.removeTerrain(staticMeshObject);
			}
			void SetAngularSpeed(float angularSpeed) {
				m_ikRigController.setAngularSpeed(angularSpeed);
			}
			IKRigController& GetIKRigController() { return m_ikRigController; }
		private:
			RigData m_rigData;
			IKRigController m_ikRigController;
	};
}






#endif