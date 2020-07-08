#include "World.hpp"
namespace Mona {

	World::World() : m_objectManager() {
		m_componentManagers[TransformComponent::componentIndex] = std::make_shared<ComponentManager<TransformComponent>>();
		m_componentManagers[CameraComponent::componentIndex] = std::make_shared<ComponentManager<CameraComponent>>();
		m_componentManagers[StaticMeshComponent::componentIndex] = std::make_shared<ComponentManager<StaticMeshComponent>>();
	}
	void World::StartUp(GameObjectID expectedObjects) noexcept {
		m_objectManager.StartUp(expectedObjects);
		for (auto& componentManager : m_componentManagers)
			componentManager->StartUp();
	}

	void World::ShutDown() noexcept {
		for (auto& componentManager : m_componentManagers)
			componentManager->Clear();
	}

	void World::Update(float timeStep) noexcept {
		m_objectManager.UpdateGameObjects(timeStep);
	}
	void World::DestroyGameObject(std::weak_ptr<GameObject> objectPointer) noexcept {
		m_objectManager.DestroyGameObject(objectPointer);
	}
	void World::DestroyGameObject(GameObjectID id) noexcept {
		m_objectManager.DestroyGameObject(id);
	}
	std::weak_ptr<GameObject> World::GetGameObject(GameObjectID id) noexcept {
		return m_objectManager.GetGameObject(id);
	}
}

