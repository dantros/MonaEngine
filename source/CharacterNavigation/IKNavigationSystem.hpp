#pragma once
#ifndef IKNAVIGATIONSYSTEM_HPP
#define IKNAVIGATIONSYSTEM_HPP

#include "../World/TransformComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"
#include "IKNavigationComponent.hpp"

namespace Mona {
	class IKRigController;
	class IKNavigationSystem {
		std::vector<IKRigController*> m_controllersDebug;
	public:
		IKNavigationSystem() = default;
		void UpdateAllRigs(ComponentManager<IKNavigationComponent>& ikNavigationManager,
			ComponentManager<TransformComponent>& transformManager,
			ComponentManager<StaticMeshComponent>& staticMeshManager,
			ComponentManager<SkeletalMeshComponent>& skeletalMeshManager, float timeStep);
		std::vector<IKRigController*> getControllersDebug() { return m_controllersDebug; }
	};
}
#endif