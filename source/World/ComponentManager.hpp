#pragma once
#ifndef COMPONENTMANAGER_HPP
#define COMPONENTMANAGER_HPP
#include "GameObjectTypes.hpp"
#include <vector>
#include <unordered_map>
#include <limits>
namespace Mona {

	class BaseComponentManager {
	public:
		using size_type = typename ComponentHandle::size_type;
		constexpr static size_type s_maxEntries = std::numeric_limits<size_type>::max();
		constexpr static size_type s_minFreeIndices = 1024;
		BaseComponentManager() = default;
		virtual ~BaseComponentManager() = default;
		virtual void StartUp(GameObjectID expectedObjects = 0) noexcept = 0;
		virtual void Clear() noexcept = 0;
		BaseComponentManager(const BaseComponentManager&) = delete;
		BaseComponentManager& operator=(const BaseComponentManager&) = delete;
	};

	template <typename ComponentType>
	class ComponentManager : public BaseComponentManager {
	public:
		ComponentManager();
		ComponentManager(const ComponentManager&) = delete;
		ComponentManager& operator=(const ComponentManager&) = delete;
		virtual void StartUp(size_type expectedObjects = 0) noexcept override;
		virtual void Clear() noexcept override;
		ComponentHandle AddComponent(const GameObjectHandle &gameObjectHandle) noexcept;
		void RemoveComponent(const ComponentHandle& handle) noexcept;
		ComponentType* GetComponentPointer(const ComponentHandle& handle) noexcept;
		const ComponentType* GetComponentPointer(const ComponentHandle& handle) const noexcept;
		size_type GetCount() const noexcept;
		GameObjectHandle GetObjectHandle(const ComponentHandle& handle) const noexcept;
		ComponentType& operator[](size_type index) noexcept;
		const ComponentType& operator[](size_type index) const noexcept;
		bool IsValid(const ComponentHandle& handle) const noexcept;

	private:
		struct HandleEntry { 
			HandleEntry(size_type i, size_type p, size_type g) : index(i), prevIndex(p), generation(g), active(true) {}
			size_type index;
			size_type prevIndex;
			size_type generation;
			bool active;
		};
		struct GameObjectEntry
		{
			GameObjectEntry(size_type h, GameObjectHandle handle) : handleEntryIndex(h), objectHandle(handle)  {}
			size_type handleEntryIndex;
			GameObjectHandle  objectHandle;
		};
		
		std::vector<ComponentType> m_components;
		std::vector<GameObjectEntry> m_gameObjects;

		std::vector<HandleEntry> m_handleEntries;
		size_type m_firstFreeIndex;
		size_type m_lastFreeIndex;
		size_type m_freeIndicesCount;
	};

	/*
	template <typename ComponentType>
	class UserComponentHandle
	{
	public:
		UserComponentHandle(ComponentHandle handle, ComponentManager<ComponentType>* manager) : m_innerHandle(handle), m_managerPtr(manager){};
	private:
		ComponentHandle m_innerHandle;
		ComponentManager<ComponentType>* m_managerPtr;
	};
	*/
}
#include "Detail/ComponentManager_Implementation.hpp"

#endif