#pragma once
#ifndef WORLD_IMPLEMENTATION_HPP
#define WORLD_IMPLEMENTATION_HPP
namespace Mona {
	template <typename ObjectType, typename ...Args>
	GameObjectHandle<ObjectType> World::CreateGameObject(Args&& ... args) noexcept
	{
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
		auto objectPointer = m_objectManager.CreateGameObject<ObjectType>(*this, std::forward<Args>(args)...);
		return GameObjectHandle<ObjectType>(objectPointer->GetInnerObjectHandle(), objectPointer);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::AddComponent(BaseGameObjectHandle& objectHandle) noexcept {
		return AddComponent<ComponentType>(*objectHandle);
	}
	template <typename ComponentType>
	ComponentHandle<ComponentType> World::AddComponent(GameObject& gameObject) noexcept {
		MONA_ASSERT(m_objectManager.IsValid(gameObject.GetInnerObjectHandle()), "World Error: Trying to add component from invaled object handle");
		//auto& go = m_objectManager.GetGameObjectReference(objectHandle);
		MONA_ASSERT(!gameObject.HasComponent<ComponentType>(),
			"Trying to add already present component");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		InnerComponentHandle componentHandle = managerPtr->AddComponent(gameObject.GetInnerObjectHandle());
		gameObject.AddInnerComponentHandle(ComponentType::componentIndex, componentHandle);
		return ComponentHandle<ComponentType>(componentHandle, managerPtr);
	}
	template <typename ComponentType>
	void World::RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept {
		auto managerPtr = std::static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		auto objectPtr = m_objectManager.GetGameObjectPointer(managerPtr->GetObjectHandle(handle.GetInnerHandle()));
		managerPtr->RemoveComponent(handle.GetInnerHandle());
		objectPtr->RemoveInnerComponentHandle(ComponentType::componentIndex);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::GetComponentHandle(const GameObject& gameObject) const noexcept
	{
		MONA_ASSERT(gameObject.HasComponent<ComponentType>(), "World Error: Object doesnt have component");
		auto managerPtr = std::static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		return ComponentHandle<ComponentType>(gameObject.GetInnerComponentHandle<ComponentType>(), managerPtr);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::GetComponentHandle(const BaseGameObjectHandle& objectHandle) const noexcept
	{
		return GetComponentHandle(*objectHandle);
	}

	template <typename ComponentType>
	BaseComponentManager::size_type World::GetComponentCount() const noexcept
	{
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		return managerPtr->GetCount();
	}

}
#endif