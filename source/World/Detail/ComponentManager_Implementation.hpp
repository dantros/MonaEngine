#pragma once
#ifndef COMPONENTMANAGER_DETAIL_HPP
#define COMPONENTMANAGER_DETAIL_HPP
#include "../../Core/Log.hpp"
namespace Mona {

/*
	template <typename ComponentType>
	bool ComponentManager<ComponentType>::ComponentHandle::IsValid() const noexcept {
		return m_managerPtr->GetComponentPointer(*this) != nullptr;
	}
	template <typename ComponentType>
	ComponentType* ComponentManager<ComponentType>::ComponentHandle::operator->() {
		auto ptr = m_managerPtr->GetComponentPointer(*this);
		MONA_ASSERT(ptr != nullptr, "WORLD: Trying to dereference Invalid Handle!");
		return ptr;
	}
	*/
	template <typename ComponentType>
	ComponentManager<ComponentType>::ComponentManager() : 
		BaseComponentManager(), 
		m_firstFreeIndex(s_maxEntries),
		m_lastFreeIndex(s_maxEntries),
		m_freeIndicesCount(0)
	{}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::StartUp(size_type expectedObjects) noexcept { 
		m_components.reserve(expectedObjects);
		m_gameObjects.reserve(expectedObjects);
		m_handleEntries.reserve(expectedObjects);
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::Clear() noexcept {
		m_components.clear();
		m_gameObjects.clear();
		m_handleEntries.clear();
		m_firstFreeIndex = s_maxEntries;
		m_lastFreeIndex = s_maxEntries;
		m_freeIndicesCount = 0;
	}

	template <typename ComponentType>
	ComponentHandle ComponentManager<ComponentType>::AddComponent(const GameObjectHandle& gameObjectHandle) noexcept
	{
		MONA_ASSERT(m_components.size() < s_maxEntries, "ComponentManager Error: Cannot Add more components, max number reached.");
		if (m_firstFreeIndex != s_maxEntries && m_freeIndicesCount > s_minFreeIndices)
		{

			//m_lookUp[gameObjectID] = m_components.size();
			auto& handleEntry = m_handleEntries[m_firstFreeIndex];
			MONA_ASSERT(handleEntry.active == false, "ComponentManager Error: Incorrect active state for handleEntry");
			MONA_ASSERT(handleEntry.generation < std::numeric_limits<decltype(handleEntry.generation)>::max(),
				"ComponentManager Error: Generational Index reached its maximunn value, component cannot be added.");

			auto handleIndex = m_firstFreeIndex;
			m_gameObjects.emplace_back(handleIndex, gameObjectHandle);
			m_components.emplace_back();
			if (m_firstFreeIndex == m_lastFreeIndex)
				m_firstFreeIndex = m_lastFreeIndex = s_maxEntries;
			else 
				m_firstFreeIndex = handleEntry.index;
			handleEntry.generation += 1;
			handleEntry.active = true;
			handleEntry.index = static_cast<size_type>(m_components.size() - 1);
			handleEntry.prevIndex = s_maxEntries;
			--m_freeIndicesCount;
			return ComponentHandle(handleIndex, handleEntry.generation);
		}
		else {
			m_handleEntries.emplace_back(static_cast<size_type>(m_components.size()), s_maxEntries, 0);
			m_gameObjects.emplace_back(static_cast<size_type>(m_handleEntries.size() - 1), gameObjectHandle);
			m_components.emplace_back();

			return ComponentHandle(static_cast<size_type>(m_handleEntries.size() - 1), 0);
		}
	}

	template <typename ComponentType>
	void  ComponentManager<ComponentType>::RemoveComponent(const ComponentHandle& handle) noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(handle.m_generation == m_handleEntries[index].generation, "ComponentManager Error: Trying to delete from invalid handle");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to delete inactive handle");
		auto& handleEntry = m_handleEntries[index];
		if (handleEntry.index < m_components.size() - 1)
		{
			m_components[handleEntry.index] = std::move(m_components.back());
			m_gameObjects[handleEntry.index] = m_gameObjects.back();
			m_handleEntries[m_gameObjects.back().handleEntryIndex].index = handleEntry.index;
		}

		m_components.pop_back();
		m_gameObjects.pop_back();

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
	template <typename ComponentType>
	ComponentType* ComponentManager<ComponentType>::GetComponentPointer(const ComponentHandle& handle) noexcept
	{
		const auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		//if (m_handleEntries[index].active == false || m_handleEntries[index].generation != handle.m_generation)
			//return nullptr;
		return &m_components[m_handleEntries[index].index];
	}

	template <typename ComponentType>
	const ComponentType* ComponentManager<ComponentType>::GetComponentPointer(const ComponentHandle& handle) const noexcept
	{
		const auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		//if (m_handleEntries[index].active == false || m_handleEntries[index].generation != handle.m_generation)
			//return nullptr;
		return &m_components[m_handleEntries[index].index];
	}

	template <typename ComponentType>
	typename ComponentManager<ComponentType>::size_type ComponentManager<ComponentType>::GetCount() const noexcept { return m_components.size(); }

	template <typename ComponentType>
	GameObjectHandle ComponentManager<ComponentType>::GetObjectHandle(const ComponentHandle& handle) const noexcept
	{
		const auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		//if (m_handleEntries[index].active == false || m_handleEntries[index].generation != handle.m_generation)
			//return INVALID_INDEX;
		return m_gameObjects[m_handleEntries[index].index].objectHandle;
	}

	template <typename ComponentType>
	ComponentType& ComponentManager<ComponentType>::operator[](ComponentManager<ComponentType>::size_type index) noexcept
	{
		MONA_ASSERT(index < m_components.size(), "ComponentManager Error: component index out of range");
		return m_components[index];
	}

	template <typename ComponentType>
	const ComponentType& ComponentManager<ComponentType>::operator[](ComponentManager<ComponentType>::size_type  index) const noexcept 
	{
		MONA_ASSERT(index < m_components.size(), "ComponentManager Error: component index out of range");
		return m_components[index];
	}

	template <typename ComponentType>
	bool ComponentManager<ComponentType>::IsValid(const ComponentHandle& handle) const noexcept
	{
		const auto index = handle.m_index;
		if (index < m_handleEntries.size() ||
			m_handleEntries[index].active == false ||
			m_handleEntries[index].generation != handle.m_generation)
			return false;
		return true;
	}



}
#endif