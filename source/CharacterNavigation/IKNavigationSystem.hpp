#pragma once
#ifndef IKNAVIGATIONSYSTEM_HPP
#define IKNAVIGATIONSYSTEM_HPP

#include "../World/TransformComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"
#include "IKNavigationComponent.hpp"

namespace Mona {
	class IKNavigationSystem {
	public:
		IKNavigationSystem() = default;
		void UpdateAllRigs(ComponentManager<IKNavigationComponent>& ikNavigationManager,
			ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager,
			ComponentManager<SkeletalMeshComponent>& skeletalMeshManager, float timeStep);
	};
}
#endif