#pragma once
#ifndef WORLD_HPP
#define WORLD_HPP
#include "GameObjectManager.hpp"
#include "Component.hpp"
#include "ComponentManager.hpp"
#include <memory>
#include <array>
#include <string>

namespace Mona {
	class World {
	public:
		World() : m_objectManager() {
			m_componentManagers[TransformComponent::componentIndex] = std::make_shared<ComponentManager<TransformComponent>>();
			m_componentManagers[CameraComponent::componentIndex] = std::make_shared<ComponentManager<CameraComponent>>();
			m_componentManagers[StaticMeshComponent::componentIndex] = std::make_shared<ComponentManager<StaticMeshComponent>>();
		}
		void StartUp(GameObjectID expectedObjects = 0) {
			m_objectManager.StartUp(expectedObjects);
			for (auto& componentManager : m_componentManagers)
				componentManager->StartUp();
		}
		void ShutDown()
		{
			for (auto& componentManager : m_componentManagers)
				componentManager->Clear();
		}
		World(const World& world) = delete;
		World& operator=(const World& world) = delete;
		static World& GetInstance() noexcept {
			static World s_world;
			return s_world;
		}
		void Update(float timeStep) noexcept {
			m_objectManager.UpdateGameObjects(timeStep);
		}
		void DestroyGameObject(std::weak_ptr<GameObject> objectPointer) noexcept {
			m_objectManager.DestroyGameObject(objectPointer);
		}
		void DestroyGameObject(GameObjectID id) noexcept {
			m_objectManager.DestroyGameObject(id);
		}
		std::weak_ptr<GameObject> GetGameObject(GameObjectID id) noexcept {
			return m_objectManager.GetGameObject(id);
		}
		template <typename ObjectType, typename ...Args>
		std::weak_ptr<GameObject> CreateGameObject(Args&& ... args)
		{
			static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
			return m_objectManager.CreateGameObject<ObjectType>(std::forward<Args>(args)...);
		}
		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(GameObjectID gameObjectID) noexcept {
			auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
			return managerPtr->AddComponent(gameObjectID);
		}
		template <typename ComponentType>
		void RemoveComponent(ComponentHandle<ComponentType> &handle) noexcept
		{
			auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
			managerPtr->RemoveComponent(handle);
		}

		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(GameObjectID gameObjectID) noexcept {
			auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
			return managerPtr->GetComponentHandle(gameObjectID);
		}
	private:
		GameObjectManager m_objectManager;
		std::array<std::shared_ptr<BaseComponentManager>, GetComponentCount()> m_componentManagers;
	};
}
#endif