#pragma once
#ifndef WORLD_HPP
#define WORLD_HPP
#include "GameObjectTypes.hpp"
#include "GameObject.hpp"
#include "GameObjectManager.hpp"
#include "Component.hpp"
#include "ComponentManager.hpp"
#include "../Event/EventManager.hpp"
#include <memory>
#include <array>
#include <string>

namespace Mona {


	class BaseGameObjectHandle;
	template <typename ObjectType>
	class GameObjectHandle;
	template <typename ComponentType>
	class ComponentHandle;
	

	class World {
	public:
		World();
		World(const World& world) = delete;
		World& operator=(const World& world) = delete;
		
		GameObjectManager::size_type GetGameObjectCount() const noexcept;
		
		bool IsValid(const InnerGameObjectHandle& handle) const noexcept;


		template <typename ObjectType, typename ...Args>
		GameObjectHandle<ObjectType> CreateGameObject(Args&& ... args) noexcept;
		void DestroyGameObject(BaseGameObjectHandle& handle) noexcept;
		void DestroyGameObject(GameObject& gameObject) noexcept;


		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(BaseGameObjectHandle& objectHandle) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(GameObject& gameObject) noexcept;
		template <typename ComponentType>
		void RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(const BaseGameObjectHandle& objectHandle) const noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(const GameObject& gameObject) const noexcept;
		template <typename ComponentType>
		BaseComponentManager::size_type GetComponentCount() const noexcept;

		EventManager& GetEventManager() const noexcept;
		void StartUp(EventManager* eventManagerPointer, GameObjectID expectedObjects = 0) noexcept;
		void ShutDown() noexcept;
		void Update(float timeStep) noexcept;

	private:
		friend class Engine;
		EventManager* m_eventManagerPointer;
		GameObjectManager m_objectManager;
		std::array<std::unique_ptr<BaseComponentManager>, GetComponentTypeCount()> m_componentManagers;
	};

	

}
#include "Detail/World_Implementation.hpp"
#endif