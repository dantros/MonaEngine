#pragma once
#ifndef COMPONENTMANAGER_IMPLEMENTATION_HPP
#define COMPONENTMANAGER_IMPLEMENTATION_HPP
#include "../../Core/Log.hpp"
#include "../../Event/EventManager.hpp"
#include "../Event/Events.hpp"
namespace Mona {

	template <typename ComponentType, typename LifetimePolicy>
	ComponentManager<ComponentType, LifetimePolicy>::ComponentManager() : 
		BaseComponentManager(), 
		m_firstFreeIndex(s_maxEntries),
		m_lastFreeIndex(s_maxEntries),
		m_freeIndicesCount(0)
	{}

	template <typename ComponentType, typename LifetimePolicy>
	void ComponentManager<ComponentType, LifetimePolicy>::StartUp(EventManager& eventManager,size_type expectedObjects) noexcept {
		m_components.reserve(expectedObjects);
		m_componentOwners.reserve(expectedObjects);
		m_handleEntryIndices.reserve(expectedObjects);
		m_handleEntries.reserve(expectedObjects);
	}

	template <typename ComponentType, typename LifetimePolicy>
	void ComponentManager<ComponentType, LifetimePolicy>::ShutDown(EventManager& eventManager) noexcept {
		for (decltype(m_components.size()) i = 0; i < m_components.size(); i++)
		{
			HandleEntry handleEntry = m_handleEntries[m_handleEntryIndices[i]];
			m_lifetimePolicy.OnRemoveComponent(m_componentOwners[i], m_components[i], InnerComponentHandle(m_handleEntryIndices[i], handleEntry.generation));
		}
		m_components.clear();
		m_componentOwners.clear();
		m_handleEntryIndices.clear();
		m_handleEntries.clear();
		m_firstFreeIndex = s_maxEntries;
		m_lastFreeIndex = s_maxEntries;
		m_freeIndicesCount = 0;
	}

	template <typename ComponentType, typename LifetimePolicy>
	template <typename ... Args>
	InnerComponentHandle ComponentManager<ComponentType, LifetimePolicy>::AddComponent(GameObject* gameObjectPointer, Args&& ... args) noexcept
	{
		MONA_ASSERT(m_components.size() < s_maxEntries, "ComponentManager Error: Cannot Add more components, max number reached.");
		if (m_firstFreeIndex != s_maxEntries && m_freeIndicesCount > s_minFreeIndices)
		{
			auto& handleEntry = m_handleEntries[m_firstFreeIndex];
			MONA_ASSERT(handleEntry.active == false, "ComponentManager Error: Incorrect active state for handleEntry");
			MONA_ASSERT(handleEntry.generation < std::numeric_limits<decltype(handleEntry.generation)>::max(),
				"ComponentManager Error: Generational Index reached its maximunn value, component cannot be added.");

			auto handleIndex = m_firstFreeIndex;
			m_componentOwners.emplace_back(gameObjectPointer);
			m_handleEntryIndices.emplace_back(handleIndex);
			m_components.emplace_back(std::forward<Args>(args)...);
			if (m_firstFreeIndex == m_lastFreeIndex)
				m_firstFreeIndex = m_lastFreeIndex = s_maxEntries;
			else 
				m_firstFreeIndex = handleEntry.index;
			handleEntry.generation += 1;
			handleEntry.active = true;
			handleEntry.index = static_cast<size_type>(m_components.size() - 1);
			handleEntry.prevIndex = s_maxEntries;
			--m_freeIndicesCount;
			InnerComponentHandle resultHandle = InnerComponentHandle(handleIndex, handleEntry.generation);
			m_lifetimePolicy.OnAddComponent(gameObjectPointer, m_components.back(), resultHandle);
			return resultHandle;
		}
		else {
			m_handleEntries.emplace_back(static_cast<size_type>(m_components.size()), s_maxEntries, 0);
			m_componentOwners.emplace_back(gameObjectPointer);
			m_handleEntryIndices.emplace_back(static_cast<size_type>(m_handleEntries.size() - 1));
			m_components.emplace_back(std::forward<Args>(args)...);
			InnerComponentHandle resultHandle = InnerComponentHandle(static_cast<size_type>(m_handleEntries.size() - 1), 0);
			m_lifetimePolicy.OnAddComponent(gameObjectPointer, m_components.back(), resultHandle);
			return resultHandle;
		}
	}

	template <typename ComponentType, typename LifetimePolicy>
	void ComponentManager<ComponentType, LifetimePolicy>::RemoveComponent(const InnerComponentHandle& handle) noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of bounds");
		MONA_ASSERT(handle.m_generation == m_handleEntries[index].generation, "ComponentManager Error: Trying to delete from invalid handle");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to delete inactive handle");
		auto& handleEntry = m_handleEntries[index];
		m_lifetimePolicy.OnRemoveComponent(m_componentOwners[handleEntry.index], m_components[handleEntry.index], handle);
		if (handleEntry.index < m_components.size() - 1)
		{
			m_components[handleEntry.index] = std::move(m_components.back());
			m_componentOwners[handleEntry.index] = m_componentOwners.back();
			m_handleEntryIndices[handleEntry.index] = m_handleEntryIndices.back();
			m_handleEntries[m_handleEntryIndices.back()].index = handleEntry.index;
		}
		
		m_components.pop_back();
		m_componentOwners.pop_back();
		m_handleEntryIndices.pop_back();

		if (m_lastFreeIndex != s_maxEntries)
			m_handleEntries[m_lastFreeIndex].index = index;
		else 
			m_firstFreeIndex = index; 
		
		handleEntry.active = false;
		handleEntry.prevIndex = m_lastFreeIndex;
		handleEntry.index = s_maxEntries;
		m_lastFreeIndex = index;
		++m_freeIndicesCount;
		
	}

	template <typename ComponentType, typename LifetimePolicy>
	ComponentType* ComponentManager<ComponentType, LifetimePolicy>::GetComponentPointer(const InnerComponentHandle& handle) noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		return &m_components[m_handleEntries[index].index];
	}
	template <typename ComponentType, typename LifetimePolicy>
	const ComponentType* ComponentManager<ComponentType, LifetimePolicy>::GetComponentPointer(const InnerComponentHandle& handle) const noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		return &m_components[m_handleEntries[index].index];
	}

	template <typename ComponentType, typename LifetimePolicy>
	typename ComponentManager<ComponentType, LifetimePolicy>::size_type ComponentManager<ComponentType, LifetimePolicy>::GetCount() const noexcept { return m_components.size(); }

	template <typename ComponentType, typename LifetimePolicy>
	GameObject* ComponentManager<ComponentType, LifetimePolicy>::GetOwner(const InnerComponentHandle& handle) const noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		return m_componentOwners[m_handleEntries[index].index];
	}


	template <typename ComponentType, typename LifetimePolicy>
	GameObject* ComponentManager<ComponentType, LifetimePolicy>::GetOwnerByIndex(size_type i) noexcept {
		MONA_ASSERT(i < m_componentOwners.size(), "ComponentManager Error: owner index out of range");
		return m_componentOwners[i];
	}
	template <typename ComponentType, typename LifetimePolicy>
	ComponentType& ComponentManager<ComponentType, LifetimePolicy>::operator[](ComponentManager<ComponentType, LifetimePolicy>::size_type index) noexcept
	{
		MONA_ASSERT(index < m_components.size(), "ComponentManager Error: component index out of range");
		return m_components[index];
	}

	template <typename ComponentType, typename LifetimePolicy>
	const ComponentType& ComponentManager<ComponentType, LifetimePolicy>::operator[](ComponentManager<ComponentType, LifetimePolicy>::size_type  index) const noexcept
	{
		MONA_ASSERT(index < m_components.size(), "ComponentManager Error: component index out of range");
		return m_components[index];
	}

	template <typename ComponentType, typename LifetimePolicy>
	bool ComponentManager<ComponentType, LifetimePolicy>::IsValid(const InnerComponentHandle& handle) const noexcept
	{
		const auto index = handle.m_index;
		if (index >= m_handleEntries.size() ||
			m_handleEntries[index].active == false ||
			m_handleEntries[index].generation != handle.m_generation)
			return false;
		return true;
	}

	template <typename ComponentType, typename LifetimePolicy>
	void ComponentManager<ComponentType, LifetimePolicy>::SetLifetimePolicy(const LifetimePolicy& policy) noexcept
	{
		m_lifetimePolicy = policy;
	}



}
#endif