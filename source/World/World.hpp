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
	template <typename ComponentType>
	class ComponentHandle {
	public:
		static constexpr uint8_t componentIndex = ComponentType::componentIndex;
		ComponentHandle() : m_innerHandle(), m_managerPointer(nullptr) {}
		ComponentHandle(InnerComponentHandle handle, ComponentManager<ComponentType>* manager) : m_innerHandle(handle), m_managerPointer(manager){}
		InnerComponentHandle GetInnerHandle() const noexcept { return m_innerHandle; }
		bool IsValid() const noexcept {
			if (m_managerPointer == nullptr)
				return false;
			return m_managerPointer->IsValid(m_innerHandle);
		}
		ComponentType* operator->()
		{
			MONA_ASSERT(IsValid(), "ComponentHandle Error: Trying to access with invalid componentHandle");
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
		const ComponentType* operator->() const {
			MONA_ASSERT(IsValid(), "ComponentHandle Error: Trying to access with invalid componentHandle");
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
	private:
		InnerComponentHandle m_innerHandle;
		ComponentManager<ComponentType>* m_managerPointer;
	};

	template <typename ObjectType>
	class GameObjectHandle {
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
	public:
		GameObjectHandle() : m_innerHandle() {}
		GameObjectHandle(InnerGameObjectHandle handle) : m_innerHandle(handle) {}
		InnerGameObjectHandle GetInnerHandle() const { return m_innerHandle; }
		bool IsValid(World& world) const noexcept {
			
			return world.IsValid(m_innerHandle);
		}
		ObjectType& operator()(World& world)
		{
			MONA_ASSERT(IsValid(world), "Trying to access with invalid objectHandle");
			return world.GetGameObjectReference<ObjectType>(m_innerHandle);
		}
		const ObjectType& operator()(World& world) const {
			MONA_ASSERT(IsValid(world), "Trying to access with invalid objectHandle");
			return world.GetGameObjectReference<ObjectType>(m_innerHandle);
		}
	private:
		InnerGameObjectHandle m_innerHandle;
	};

	class World {
	public:
		World();
		World(const World& world) = delete;
		World& operator=(const World& world) = delete;
		
		GameObjectManager::size_type GetGameObjectCount() const noexcept;
		void DestroyGameObject(const InnerGameObjectHandle& handle) noexcept;
		bool IsValid(const InnerGameObjectHandle& handle) const noexcept;

		template <typename ObjectType>
		ObjectType& GetGameObjectReference(const InnerGameObjectHandle& handle) noexcept;
		template <typename ObjectType, typename ...Args>
		InnerGameObjectHandle CreateGameObject(Args&& ... args) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(const InnerGameObjectHandle &objectHandle) noexcept;
		template <typename ComponentType>
		void RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept;
		template <typename ComponentType>
		InnerComponentHandle GetComponentHandle(const InnerGameObjectHandle& objectHandle) const noexcept;
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

	using TransformHandle = ComponentHandle<TransformComponent>;
	using StaticMeshHandle = ComponentHandle<StaticMeshComponent>;
	using CameraHandle = ComponentHandle<CameraComponent>;


	template <typename ObjectType, typename ... Args>
	GameObjectHandle<ObjectType> CreateGameObject(World& world, Args&& ... args) noexcept
	{
		return GameObjectHandle<ObjectType>(world.CreateGameObject<ObjectType>(std::forward<Args>(args)...));
	}
	template <typename ObjectType>
	void DestroyGameObject(World& world, GameObjectHandle<ObjectType> objectHandle) noexcept
	{
		world.DestroyGameObject(objectHandle.GetInnerHandle());
	}

}
#include "Detail/World_Implementation.hpp"
#endif