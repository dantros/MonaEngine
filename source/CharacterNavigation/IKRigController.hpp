#ifndef IKRIGCONTROLLER_HPP
#define IKRIGCONTROLLER_HPP

#include "IKRig.hpp"

namespace Mona {

	class AnimationController;
	class IKRigController {
		friend class IKNavigationComponent;
		IKRig m_ikRig;
		AnimationController* m_animationController;
	public:
		IKRigController() = default;
		IKRigController(AnimationController* animController, IKRig ikRig);
		void addAnimation(std::shared_ptr<AnimationClip> animationClip);
		int removeAnimation(std::shared_ptr<AnimationClip> animationClip);
		void updateIKRigConfigTime(float time, AnimationIndex animIndex);
		void updateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>* transformManager,
			ComponentManager<StaticMeshComponent>* staticMeshManager);
		void updateAnimation(AnimationIndex animIndex);
		void updateIKRig(float time, ComponentManager<TransformComponent>* transformManager,
			ComponentManager<StaticMeshComponent>* staticMeshManager);
		void updateFrontVector(float time);
	};








}



#endif