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
		ComponentHandle<ComponentType> AddComponent(GameObjectID gameObjectID) noexcept;
		template <typename ComponentType>
		void RemoveComponent(ComponentHandle<ComponentType>& handle) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(GameObjectID gameObjectID) noexcept;
		template <typename ComponentType>
		ComponentType* GetComponentPointer(ComponentHandle<ComponentType>& handle) noexcept;

	private:
		GameObjectManager m_objectManager;
		std::array<std::shared_ptr<BaseComponentManager>, GetComponentCount()> m_componentManagers;

	};

		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(GameObject &object) noexcept{
			return World::GetInstance().AddComponent<ComponentType>(object.GetObjectID());
		}

		template <typename ComponentType>
		void RemoveComponent(ComponentHandle<ComponentType>& handle) {
			World::GetInstance().RemoveComponent(handle);
		}


}
#include "Detail/World.hpp"
#endif