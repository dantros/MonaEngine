#pragma once
#ifndef WORLD_IMPLEMENTATION_HPP
#define WORLD_IMPLEMENTATION_HPP
namespace Mona {
	template <typename ObjectType, typename ...Args>
	InnerGameObjectHandle World::CreateGameObject(Args&& ... args) noexcept
	{
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
		return m_objectManager.CreateGameObject<ObjectType>(*this, std::forward<Args>(args)...);
	}
	template <typename ObjectType>
	ObjectType& World::GetGameObjectReference(const InnerGameObjectHandle& handle) noexcept {
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
		return static_cast<ObjectType&>(m_objectManager.GetGameObjectReference(handle));
	}
	template <typename ComponentType>
	ComponentHandle<ComponentType> World::AddComponent(const InnerGameObjectHandle& objectHandle) noexcept {
		MONA_ASSERT(m_objectManager.IsValid(objectHandle), "World Error: Trying to add component from invaled object handle");
		auto& go = m_objectManager.GetGameObjectReference(objectHandle);
		MONA_ASSERT(go.m_componentHandles.find(ComponentType::componentIndex) == go.m_componentHandles.end(),
			"Trying to add already present component");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		InnerComponentHandle componentHandle = managerPtr->AddComponent(objectHandle);
		go.m_componentHandles[ComponentType::componentIndex] = componentHandle;
		return ComponentHandle<ComponentType>(componentHandle, managerPtr);
	}

	template <typename ComponentType>
	void World::RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept {
		auto managerPtr = std::static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		managerPtr->RemoveComponent(handle.GetInnerHandle());
		auto& go = m_objectManager.GetGameObjectReference(objectHandle);
		go.m_componentHandles.remove(ComponentType::componentIndex);
	}

	template <typename ComponentType>
	InnerComponentHandle World::GetComponentHandle(const InnerGameObjectHandle& objectHandle) const noexcept
	{
		return InnerComponentHandle();
	}

	template <typename ComponentType>
	BaseComponentManager::size_type World::GetComponentCount() const noexcept
	{
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		return managerPtr->GetCount();
	}

}
#endif