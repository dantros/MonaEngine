#include "World.hpp"
namespace Mona {
	
	World::World() : m_objectManager() {
		m_eventManagerPointer = nullptr;
		m_componentManagers[TransformComponent::componentIndex].reset(new ComponentManager<TransformComponent>());
		m_componentManagers[CameraComponent::componentIndex].reset(new ComponentManager<CameraComponent>());
		m_componentManagers[StaticMeshComponent::componentIndex].reset(new ComponentManager<StaticMeshComponent>());
	}
	void World::StartUp(EventManager* eventManagerPointer, GameObjectID expectedObjects) noexcept {
		m_eventManagerPointer = eventManagerPointer;
		m_objectManager.StartUp(expectedObjects);
		for (auto& componentManager : m_componentManagers)
			componentManager->StartUp(*m_eventManagerPointer, expectedObjects);
	}

	void World::ShutDown() noexcept {
		m_objectManager.ShutDown(*this);
		for (auto& componentManager : m_componentManagers)
			componentManager->ShutDown(*m_eventManagerPointer);
	}

	void World::Update(float timeStep) noexcept {
		m_objectManager.UpdateGameObjects(*this, timeStep);
	}

	void World::DestroyGameObject(BaseGameObjectHandle& handle) noexcept {
		DestroyGameObject(*handle);
	}

	void World::DestroyGameObject(GameObject& gameObject) noexcept {
		m_objectManager.DestroyGameObject(*this, gameObject.GetInnerObjectHandle());
	}

	bool World::IsValid(const InnerGameObjectHandle& handle) const noexcept {
		return m_objectManager.IsValid(handle);
	}
	GameObjectManager::size_type World::GetGameObjectCount() const noexcept
	{
		return m_objectManager.GetCount();
	}
	EventManager& World::GetEventManager() const noexcept {
		return *m_eventManagerPointer;
	}
}

