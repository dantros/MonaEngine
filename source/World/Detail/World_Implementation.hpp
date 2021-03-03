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

	template <typename ComponentType, typename ...Args>
	ComponentHandle<ComponentType> World::AddComponent(BaseGameObjectHandle& objectHandle, Args&& ... args) noexcept {
		return AddComponent<ComponentType>(*objectHandle, std::forward<Args>(args)...);
	}
	template <typename ComponentType, typename ...Args>
	ComponentHandle<ComponentType> World::AddComponent(GameObject& gameObject, Args&& ... args) noexcept {
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		MONA_ASSERT(m_objectManager.IsValid(gameObject.GetInnerObjectHandle()), "World Error: Trying to add component from invaled object handle");
		MONA_ASSERT(!gameObject.HasComponent<ComponentType>(),
			"World Error: Trying to add already present component. ComponentType = {0}", ComponentType::componentName);
		MONA_ASSERT(CheckDependencies<ComponentType>(gameObject, typename ComponentType::dependencies()), "World Error: Trying to add component with incomplete dependencies");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		InnerComponentHandle componentHandle = managerPtr->AddComponent(&gameObject, std::forward<Args>(args)...);
		gameObject.AddInnerComponentHandle(ComponentType::componentIndex, componentHandle);
		return ComponentHandle<ComponentType>(componentHandle, managerPtr);

	}
	template <typename ComponentType>
	void World::RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept {
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		auto objectPtr = m_objectManager.GetGameObjectPointer(managerPtr->GetOwner(handle.GetInnerHandle()));
		managerPtr->RemoveComponent(handle.GetInnerHandle());
		objectPtr->RemoveInnerComponentHandle(ComponentType::componentIndex);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::GetComponentHandle(const GameObject& gameObject) const noexcept
	{
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		MONA_ASSERT(gameObject.HasComponent<ComponentType>(), "World Error: Object doesnt have component of type : {0}", ComponentType::componentName);
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		return ComponentHandle<ComponentType>(gameObject.GetInnerComponentHandle<ComponentType>(), managerPtr);
	}

	template <typename ComponentType>
	ComponentHandle<ComponentType> World::GetComponentHandle(const BaseGameObjectHandle& objectHandle) const noexcept
	{
		return GetComponentHandle<ComponentType>(*objectHandle);
	}

	template <typename ComponentType>
	BaseComponentManager::size_type World::GetComponentCount() const noexcept
	{
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		return managerPtr->GetCount();
	}

	template <typename SiblingType, typename ComponentType>
	ComponentHandle<SiblingType> World::GetSiblingComponentHandle(const ComponentHandle<ComponentType>& handle) noexcept
	{
		static_assert(is_component<SiblingType>, "Template parameter is not a component");
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		auto siblingManagerPtr = static_cast<ComponentManager<SiblingType>*>(m_componentManagers[SiblingType::componentIndex].get());
		GameObject* gameObject = managerPtr->GetOwner(handle.GetInnerHandle());
		return  ComponentHandle<SiblingType>(gameObject->GetInnerComponentHandle<SiblingType>(), siblingManagerPtr);
	}

	template <typename ComponentType>
	BaseGameObjectHandle World::GetOwner(const ComponentHandle<ComponentType>& handle) noexcept {
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		auto managerPtr = static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
		GameObject* gameObject = managerPtr->GetOwner(handle.GetInnerHandle());
		return BaseGameObjectHandle(gameObject->GetInnerObjectHandle(), gameObject);
	}

	template <typename ComponentType>
	auto& World::GetComponentManager() noexcept {
		static_assert(is_component<ComponentType>, "Template parameter is not a component");
		return *static_cast<ComponentManager<ComponentType>*>(m_componentManagers[ComponentType::componentIndex].get());
	}

	template <typename ComponentType, typename ... ComponentTypes>
	bool World::CheckDependencies(const GameObject& gameObject, DependencyList<ComponentTypes...> dl) const {
		return ((gameObject.HasComponent<ComponentTypes>() ? 
			true : 
			(Log::GetLogger()->error("World Error: Component {0} depends on {1}, please add it first", ComponentType::componentName, ComponentTypes::componentName), false) ) 
			&& ...);
	
	}


}
#endif