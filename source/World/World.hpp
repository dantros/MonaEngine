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

	class BaseGameObjectHandle;
	template <typename ObjectType>
	class GameObjectHandle;
	

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

	using TransformHandle = ComponentHandle<TransformComponent>;
	using StaticMeshHandle = ComponentHandle<StaticMeshComponent>;
	using CameraHandle = ComponentHandle<CameraComponent>;

	class BaseGameObjectHandle {
	public:
		BaseGameObjectHandle() : m_innerHandle(), m_objectPointer(nullptr){}
		BaseGameObjectHandle(InnerGameObjectHandle handle, GameObject* object) : m_innerHandle(handle), m_objectPointer(object) {}
		bool IsValid(World& world) const noexcept {
			return world.IsValid(m_innerHandle);
		}
		const GameObject* operator->() const {
			return m_objectPointer;
		}
		GameObject* operator->() {
			return m_objectPointer;
		}
		GameObject& operator*() {
			return *m_objectPointer;
		}

		const  GameObject& operator*() const {
			return *m_objectPointer;
		}
		InnerGameObjectHandle GetInnerHandle() const { return m_innerHandle; }
	protected:
		GameObject* m_objectPointer;
	private:
		InnerGameObjectHandle m_innerHandle;
	};

	template <typename ObjectType>
	class GameObjectHandle : public BaseGameObjectHandle {
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
	public:
		GameObjectHandle() : BaseGameObjectHandle() {}
		GameObjectHandle(InnerGameObjectHandle handle, ObjectType* object) : BaseGameObjectHandle(handle, object) {}
		const ObjectType* operator->() const {
			return static_cast<ObjectType*>(m_objectPointer);
		}
		ObjectType* operator->() {
			return static_cast<ObjectType*>(m_objectPointer);
		}
		ObjectType& operator*() {
			return static_cast<ObjectType&>(*m_objectPointer);
		}

		const ObjectType& operator*() const {
			return static_cast<ObjectType&>(*m_objectPointer);
		}
	};

}
#include "Detail/World_Implementation.hpp"
#endif