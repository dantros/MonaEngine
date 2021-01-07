#include "AnimationSystem.hpp"
#include "../World/ComponentManager.hpp"
namespace Mona {
	void AnimationSystem::UpdateAllPoses(SkeletalMeshComponent::managerType& skeletalMeshDataManager, float timeStep) noexcept {
		for (uint32_t i = 0; i < skeletalMeshDataManager.GetCount(); i++) {
			SkeletalMeshComponent& skeletalMesh = skeletalMeshDataManager[i];
			auto& animationController = skeletalMesh.GetAnimationController();
			animationController.UpdateCurrentPose(timeStep);
		}
	}
}