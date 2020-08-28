#pragma once
#ifndef COMPONENTMANAGER_HPP
#define COMPONENTMANAGER_HPP
#include "GameObjectTypes.hpp"
#include "Component.hpp"
#include "../Core/Log.hpp"
#include "../Event/Events.hpp"
#include <vector>
#include <unordered_map>
#include <limits>
namespace Mona {
	class EventManager;
	class BaseComponentManager {
	public:
		using size_type = typename InnerComponentHandle::size_type;
		constexpr static size_type s_maxEntries = std::numeric_limits<size_type>::max();
		constexpr static size_type s_minFreeIndices = 1024;
		BaseComponentManager() = default;
		virtual ~BaseComponentManager() = default;
		virtual void StartUp(EventManager& eventManager,size_type expectedObjects = 0) noexcept = 0;
		virtual void ShutDown(EventManager& eventManager) noexcept = 0;
		virtual void RemoveComponent(const InnerComponentHandle& handle) = 0;
		BaseComponentManager(const BaseComponentManager&) = delete;
		BaseComponentManager& operator=(const BaseComponentManager&) = delete;
	};

	template <typename ComponentType>
	class ComponentManager : public BaseComponentManager {
	public:
		ComponentManager();
		ComponentManager(const ComponentManager&) = delete;
		ComponentManager& operator=(const ComponentManager&) = delete;
		virtual void StartUp(EventManager& eventManager, size_type expectedObjects = 0) noexcept override;
		virtual void ShutDown(EventManager& eventManager) noexcept override;
		InnerComponentHandle AddComponent(GameObject* gameObjectPointer) noexcept;
		virtual void RemoveComponent(const InnerComponentHandle& handle) noexcept override;
		ComponentType* GetComponentPointer(const InnerComponentHandle& handle) noexcept;
		const ComponentType* GetComponentPointer(const InnerComponentHandle& handle) const noexcept;
		size_type GetCount() const noexcept;
		GameObject* GetOwner(const InnerComponentHandle& handle) const noexcept;
		GameObject* GetOwnerByIndex(size_type i) noexcept;
		ComponentType& operator[](size_type index) noexcept;
		const ComponentType& operator[](size_type index) const noexcept;
		bool IsValid(const InnerComponentHandle& handle) const noexcept;

	private:
		struct HandleEntry { 
			HandleEntry(size_type i, size_type p, size_type g) : index(i), prevIndex(p), generation(g), active(true) {}
			size_type index;
			size_type prevIndex;
			size_type generation;
			bool active;
		};
		
		std::vector<ComponentType> m_components;
		std::vector<GameObject*> m_componentOwners;
		
		std::vector<uint32_t> m_handleEntryIndices;
		std::vector<HandleEntry> m_handleEntries;

		size_type m_firstFreeIndex;
		size_type m_lastFreeIndex;
		size_type m_freeIndicesCount;
	};

}
#include "Detail/ComponentManager_Implementation.hpp"

#endif