#pragma once
#ifndef WORLD_DETAIL_HPP
#define WORLD_DETAIL_HPP
namespace Mona {

	template <typename ObjectType, typename ...Args>
	std::weak_ptr<GameObject> World::CreateGameObject(Args&& ... args) noexcept
	{
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
		return m_objectManager.CreateGameObject<ObjectType>(std::forward<Args>(args)...);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::AddComponent(GameObjectID gameObjectID) noexcept {
		auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
		return managerPtr->AddComponent(gameObjectID);
	}

	template <typename ComponentType>
	void World::RemoveComponent(ComponentHandle<ComponentType>& handle) noexcept
	{
		auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
		managerPtr->RemoveComponent(handle);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::GetComponentHandle(GameObjectID gameObjectID) noexcept {
		auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
		return managerPtr->GetComponentHandle(gameObjectID);
	}

	template <typename ComponentType>
	ComponentType* World::GetComponentPointer(ComponentHandle<ComponentType>& handle) noexcept {
		auto managerPtr = std::static_pointer_cast<ComponentManager<ComponentType>>(m_componentManagers[ComponentType::componentIndex]);
		return managerPtr->GetComponentPointer(handle);
	}

}
#endif