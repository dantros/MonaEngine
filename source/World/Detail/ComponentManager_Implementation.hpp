#pragma once
#ifndef COMPONENTMANAGER_IMPLEMENTATION_HPP
#define COMPONENTMANAGER_IMPLEMENTATION_HPP
#include "../../Core/Log.hpp"
#include "../../Event/EventManager.hpp"
#include "../../Event/Events.hpp"
namespace Mona {

	template <typename ComponentType>
	ComponentManager<ComponentType>::ComponentManager() : 
		BaseComponentManager(), 
		m_firstFreeIndex(s_maxEntries),
		m_lastFreeIndex(s_maxEntries),
		m_freeIndicesCount(0)
	{}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::StartUp(EventManager& eventManager,size_type expectedObjects) noexcept {
		m_components.reserve(expectedObjects);
		m_componentOwners.reserve(expectedObjects);
		m_handleEntryIndices.reserve(expectedObjects);
		m_handleEntries.reserve(expectedObjects);
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::ShutDown(EventManager& eventManager) noexcept {
		//Antes de limpiar las componentes es necesario llamar OnRemoveComponent por temas de liberación de recursos por ejemplo
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

	template <typename ComponentType>
	template <typename ... Args>
	InnerComponentHandle ComponentManager<ComponentType>::AddComponent(GameObject* gameObjectPointer, Args&& ... args) noexcept
	{
		MONA_ASSERT(m_components.size() < s_maxEntries, "ComponentManager Error: Cannot Add more components, max number reached.");
		//Si hay una entrada libre en el arreglo de HandleEntry y en cantidad son mayores que s_minFreeIndices entonces se reusa una
		if (m_firstFreeIndex != s_maxEntries && m_freeIndicesCount > s_minFreeIndices)
		{
			auto& handleEntry = m_handleEntries[m_firstFreeIndex];
			MONA_ASSERT(handleEntry.active == false, "ComponentManager Error: Incorrect active state for handleEntry");
			MONA_ASSERT(handleEntry.generation < std::numeric_limits<decltype(handleEntry.generation)>::max(),
				"ComponentManager Error: Generational Index reached its maximunn value, component cannot be added.");

			auto handleIndex = m_firstFreeIndex;
			m_componentOwners.emplace_back(gameObjectPointer);
			m_handleEntryIndices.emplace_back(handleIndex);
			//Se agrega la componente al final del arreglo usando perfect forwarding para evitar copias innecesarias.
			m_components.emplace_back(std::forward<Args>(args)...);
			//Se actualiza la estructura de datos para mantener consistencia
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
			//En caso de que no hayan entradas libres o estas sean menor que s_minFreeIndices
			//Simplemente agregamos una entrada al final
			m_handleEntries.emplace_back(static_cast<size_type>(m_components.size()), s_maxEntries, 0);
			m_componentOwners.emplace_back(gameObjectPointer);
			m_handleEntryIndices.emplace_back(static_cast<size_type>(m_handleEntries.size() - 1));
			m_components.emplace_back(std::forward<Args>(args)...);
			InnerComponentHandle resultHandle = InnerComponentHandle(static_cast<size_type>(m_handleEntries.size() - 1), 0);
			m_lifetimePolicy.OnAddComponent(gameObjectPointer, m_components.back(), resultHandle);
			return resultHandle;
		}
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::RemoveComponent(const InnerComponentHandle& handle) noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of bounds");
		MONA_ASSERT(handle.m_generation == m_handleEntries[index].generation, "ComponentManager Error: Trying to delete from invalid handle");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to delete inactive handle");
		auto& handleEntry = m_handleEntries[index];
		m_lifetimePolicy.OnRemoveComponent(m_componentOwners[handleEntry.index], m_components[handleEntry.index], handle);
		//Si la componente a eliminar no es la ultima, las intercambiamos entre estas (la ultima y la que se eliminara)
		//Asi en ambos casos la componente a eliminar quedara al final del arreglo => eliminación rapida
		if (handleEntry.index < m_components.size() - 1)
		{
			m_components[handleEntry.index] = std::move(m_components.back());
			m_componentOwners[handleEntry.index] = m_componentOwners.back();
			m_handleEntryIndices[handleEntry.index] = m_handleEntryIndices.back();
			m_handleEntries[m_handleEntryIndices.back()].index = handleEntry.index;
		}
		
		//Se Elimina el ultimo elemento
		m_components.pop_back();
		m_componentOwners.pop_back();
		m_handleEntryIndices.pop_back();

		//Se Actualiza el resto de la estrutura de datos para mantener consistencia
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
	ComponentType* ComponentManager<ComponentType>::GetComponentPointer(const InnerComponentHandle& handle) noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		return &m_components[m_handleEntries[index].index];
	}
	template <typename ComponentType>
	const ComponentType* ComponentManager<ComponentType>::GetComponentPointer(const InnerComponentHandle& handle) const noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		return &m_components[m_handleEntries[index].index];
	}

	template <typename ComponentType>
	typename ComponentManager<ComponentType>::size_type ComponentManager<ComponentType>::GetCount() const noexcept { return m_components.size(); }

	template <typename ComponentType>
	GameObject* ComponentManager<ComponentType>::GetOwner(const InnerComponentHandle& handle) const noexcept
	{
		auto index = handle.m_index;
		MONA_ASSERT(index < m_handleEntries.size(), "ComponentManager Error: handle index out of range");
		MONA_ASSERT(m_handleEntries[index].active == true, "ComponentManager Error: Trying to access inactive handle");
		MONA_ASSERT(m_handleEntries[index].generation == handle.m_generation, "ComponentManager Error: handle with incorrect generation");
		return m_componentOwners[m_handleEntries[index].index];
	}


	template <typename ComponentType>
	GameObject* ComponentManager<ComponentType>::GetOwnerByIndex(size_type i) noexcept {
		MONA_ASSERT(i < m_componentOwners.size(), "ComponentManager Error: owner index out of range");
		return m_componentOwners[i];
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
	bool ComponentManager<ComponentType>::IsValid(const InnerComponentHandle& handle) const noexcept
	{
		const auto index = handle.m_index;
		if (index >= m_handleEntries.size() ||
			m_handleEntries[index].active == false ||
			m_handleEntries[index].generation != handle.m_generation)
			return false;
		return true;
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::SwapComponents(size_type first, size_type second) noexcept {
		MONA_ASSERT(first < m_components.size(), "ComponentManager Error: component index out of range");
		MONA_ASSERT(second < m_components.size(), "ComponentManager Error: component index out of range");
		if (first == second) return;
		m_handleEntries[m_handleEntryIndices[first]].index = second;
		m_handleEntries[m_handleEntryIndices[second]].index = first;
		std::swap(m_handleEntryIndices[first], m_handleEntryIndices[second]);
		std::swap(m_componentOwners[first], m_componentOwners[second]);
		std::swap(m_components[first], m_components[second]);
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::SetLifetimePolicy(const typename ComponentType::LifetimePolicyType& policy) noexcept
	{
		m_lifetimePolicy = policy;
	}



}
#endif