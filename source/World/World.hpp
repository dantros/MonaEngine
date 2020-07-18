#pragma once
#ifndef WORLD_HPP
#define WORLD_HPP
#include "GameObjectTypes.hpp"
#include "GameObject.hpp"
#include "GameObjectManager.hpp"
#include "Component.hpp"
#include "ComponentManager.hpp"
#include <memory>
#include <array>
#include <string>

namespace Mona {
	
	class World {
	public:
		template <typename ComponentType>
		friend UserComponentHandle<ComponentType> AddComponent(World& world, GameObject& object); 
		static World& GetInstance() noexcept {
			static World s_world;
			return s_world;
		}
		World();
		void StartUp(GameObjectID expectedObjects = 0) noexcept;
		void ShutDown() noexcept;
		World(const World& world) = delete;
		World& operator=(const World& world) = delete;
		void Update(float timeStep) noexcept;
		void DestroyGameObject(std::weak_ptr<GameObject> objectPointer) noexcept;
		void DestroyGameObject(GameObjectID id) noexcept;
		std::weak_ptr<GameObject> GetGameObject(GameObjectID id) noexcept;
		
		template <typename ObjectType, typename ...Args>
		std::weak_ptr<GameObject> CreateGameObject(Args&& ... args) noexcept;
		template <typename ComponentType>
		ComponentHandle AddComponent(GameObjectID gameObjectID) noexcept;
		template <typename ComponentType>
		void RemoveComponent(const ComponentHandle& handle) noexcept;
		template <typename ComponentType>
		ComponentHandle GetComponentHandle(GameObjectID gameObjectID) noexcept;
		template <typename ComponentType>
		ComponentType* GetComponentPointer(const ComponentHandle& handle) noexcept;

	private:
		GameObjectManager m_objectManager;
		template <typename ComponentType>
		ComponentManager<ComponentType>* GetManagerPointer() noexcept;
		std::array<std::shared_ptr<BaseComponentManager>, GetComponentCount()> m_componentManagers;

	};
}
#include "Detail/World_Implementation.hpp"
#endif