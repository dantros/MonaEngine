#pragma once
#ifndef COMPONENTMANAGER_DETAIL_HPP
#define COMPONENTMANAGER_DETAIL_HPP
#include "../Core/Log.hpp"
namespace Mona {
	template <typename ComponentType>
	ComponentManager<ComponentType>::ComponentManager() : BaseComponentManager() {}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::StartUp(GameObjectID expectedObjects) noexcept { 
		m_components.reserve(expectedObjects);
		m_gameObjects.reserve(expectedObjects);
		m_lookUp.reserve(expectedObjects);
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::Clear() noexcept {
		m_components.clear();
		m_gameObjects.clear();
		m_lookUp.clear();
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> ComponentManager<ComponentType>::AddComponent(GameObjectID gameObjectID) noexcept {
		MONA_ASSERT(gameObjectID != INVALID_INDEX, "ComponentManager Error: Trying to add Component to invalid GameObject");
		MONA_ASSERT(m_lookUp.find(gameObjectID) == m_lookUp.end(), "ComponentManager Error: Trying to add component to entity that already has one");
		m_lookUp[gameObjectID] = m_components.size();
		m_components.emplace_back();
		m_gameObjects.push_back(gameObjectID);
		return ComponentHandle(this, gameObjectID);
	}

	template <typename ComponentType>
	void ComponentManager<ComponentType>::RemoveComponent(ComponentHandle<ComponentType>& handle) noexcept
	{
		auto it = m_lookUp.find(handle.GetOwnerID());
		if (it != m_lookUp.end())
		{
			const size_t index = it->second;
			if (index < m_components.size() - 1)
			{
				m_components[index] = std::move(m_components.back());
				m_gameObjects[index] = m_gameObjects.back();
				m_lookUp[m_gameObjects[index]] = index;
			}
			m_components.pop_back();
			m_gameObjects.pop_back();
			m_lookUp.erase(gameObjectID);
		}
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> ComponentManager<ComponentType>::GetComponentHandle(GameObjectID gameObjectID) noexcept {
		auto it = m_lookUp.find(gameObjectID);
		if (it != m_lookUp.end())
		{
			//return it->second;
			return ComponentHandle(this, gameObjectID);
		}
		return ComponentHandle(this, INVALID_INDEX);
	}

	template <typename ComponentType>
	ComponentType* ComponentManager<ComponentType>::GetComponentPtr(ComponentHandle<ComponentType>& handle) noexcept {
		auto it = m_lookUp.find(handle.GetOwnerID());
		if (it != m_lookUp.end())
		{
			return &m_components[it->second];
		}
		return nullptr;
	}

	template <typename ComponentType>
	typename ComponentManager<ComponentType>::size_type ComponentManager<ComponentType>::GetCount() const noexcept { return m_components.size(); }

	template <typename ComponentType>
	GameObjectID ComponentManager<ComponentType>::GetObjectID(ComponentManager<ComponentType>::size_type index) const noexcept { return m_gameObjects[index]; }

	template <typename ComponentType>
	ComponentType& ComponentManager<ComponentType>::operator[](ComponentManager<ComponentType>::size_type index) noexcept { return m_components[index]; }

	template <typename ComponentType>
	const ComponentType& ComponentManager<ComponentType>::operator[](ComponentManager<ComponentType>::size_type  index) const noexcept { return m_components[index]; }



}
#endif