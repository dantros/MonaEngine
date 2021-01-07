#pragma once
#ifndef ANIMATIONSYSTEM_HPP
#define ANIMATIONSYSTEM_HPP
#include "SkeletalMeshComponent.hpp"
namespace Mona {
	class AnimationSystem {
	public:
		AnimationSystem() = default;
		void UpdateAllPoses(SkeletalMeshComponent::managerType& skeletalMeshDataManager, float timeStep) noexcept;
	};
}
#endif