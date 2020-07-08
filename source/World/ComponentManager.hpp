#pragma once
#ifndef COMPONENTMANAGER_HPP
#define COMPONENTMANAGER_HPP
#include "GameObject.hpp"
#include "ComponentHandle.hpp"
#include <vector>
#include <unordered_map>
namespace Mona {

	class BaseComponentManager {
	public:
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
		using size_type = typename std::vector<ComponentType>::size_type;
		ComponentManager();
		ComponentManager(const ComponentManager&) = delete;
		ComponentManager& operator=(const ComponentManager&) = delete;
		virtual void StartUp(GameObjectID expectedObjects = 0) noexcept override;
		virtual void Clear() noexcept override;
		ComponentHandle<ComponentType> AddComponent(GameObjectID gameObjectID) noexcept;
		void RemoveComponent(ComponentHandle<ComponentType>& handle) noexcept;
		ComponentHandle<ComponentType> GetComponentHandle(GameObjectID gameObjectID) noexcept;
		ComponentType* GetComponentPointer(ComponentHandle<ComponentType>& handle) noexcept;
		size_type GetCount() const noexcept;
		GameObjectID GetObjectID(size_type index) const noexcept;
		ComponentType& operator[](size_type index) noexcept;
		const ComponentType& operator[](size_type index) const noexcept;

	private:
		std::vector<ComponentType> m_components;
		std::vector<GameObjectID> m_gameObjects;
		std::unordered_map<GameObjectID, size_t> m_lookUp;
	};

}
#include "Detail/ComponentManager.hpp"

#endif