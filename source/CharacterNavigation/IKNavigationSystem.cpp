#include "IKNavigationSystem.hpp"
#include "../World/ComponentManager.hpp"
#include "IKNavigationLifetimePolicy.hpp"

namespace Mona {

	void IKNavigationSystem::UpdateAllRigs(ComponentManager<IKNavigationComponent>& ikNavigationManager,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager, 
		ComponentManager<SkeletalMeshComponent>& skeletalMeshManager, float timeStep) {
		for (uint32_t i = 0; i < ikNavigationManager.GetCount(); i++) {
			IKNavigationComponent& ikNav = ikNavigationManager[i];
			IKRigController& ikRigController = ikNav.GetIKRigController();
			ikRigController.updateIKRig(timeStep, transformManager, staticMeshManager, skeletalMeshManager);
			#if NDEBUG
			#else
			if (funcUtils::findIndex(m_controllersDebug, &ikRigController) == -1) {
				m_controllersDebug.push_back(&ikRigController);
			}
			#endif
		}
	}



}
