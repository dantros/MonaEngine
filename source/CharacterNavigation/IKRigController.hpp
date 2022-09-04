#pragma once
#ifndef IKRIGCONTROLLER_HPP
#define IKRIGCONTROLLER_HPP

#include "IKRig.hpp"
#include "AnimationValidator.hpp"
#include "../World/ComponentManager.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"

namespace Mona {

	class AnimationController;
	class IKRigController {
		friend class IKNavigationComponent;
		friend class DebugDrawingSystem_ikNav;
		IKRig m_ikRig;
		InnerComponentHandle m_skeletalMeshHandle;
		glm::mat4 m_baseGlobalTransform;
		AnimationValidator m_animationValidator;
		float m_reproductionTime = 0;
		bool m_ikEnabled;
		bool m_transitioning;
	public:
		IKRigController() = default;
		IKRigController(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle,
			InnerComponentHandle skeletalMeshHandle, ComponentManager<TransformComponent>* transformManager);
		void validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager);
		void addAnimation(std::shared_ptr<AnimationClip> animationClip, AnimationType animationType);
		void setAngularSpeed(float angularSpeed) { m_ikRig.setAngularSpeed(angularSpeed); }
		AnimationIndex removeAnimation(std::shared_ptr<AnimationClip> animationClip);
		void updateIKRigConfigTime(float animationTimeStep, AnimationIndex animIndex, AnimationController& animController);
		void updateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager);
		void updateGlobalTransform(ComponentManager<TransformComponent>& transformManager);
		void updateAnimation(AnimationIndex animIndex);
		void updateIKRig(float timeStep, ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager, ComponentManager<SkeletalMeshComponent>& skeletalMeshManager);
		void updateMovementDirection(float timeStep);
		void clearConfig(IKRigConfig& config);
		void enableIK(bool enableIK);
		void init();
	};








}



#endif