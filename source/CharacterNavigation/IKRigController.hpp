#pragma once
#ifndef IKRIGCONTROLLER_HPP
#define IKRIGCONTROLLER_HPP

#include "IKRig.hpp"
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
		float m_reproductionTime = 0;
	public:
		IKRigController() = default;
		IKRigController(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle transformHandle,
			InnerComponentHandle skeletalMeshHandle, ComponentManager<TransformComponent>* transformManager);
		void validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager);
		void addAnimation(std::shared_ptr<AnimationClip> animationClip);
		void setAngularSpeed(float angularSpeed) { m_ikRig.setAngularSpeed(angularSpeed); }
		AnimationIndex removeAnimation(std::shared_ptr<AnimationClip> animationClip);
		void updateIKRigConfigTime(float animationTimeStep, AnimationIndex animIndex);
		void updateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager);
		void updateAnimation(AnimationIndex animIndex);
		void updateIKRig(float timeStep, ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager, ComponentManager<SkeletalMeshComponent>& skeletalMeshManager);
		void updateMovementDirection(float timeStep);
		void init();
	};








}



#endif