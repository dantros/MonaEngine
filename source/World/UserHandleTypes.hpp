#pragma once
#ifndef USERHANDLETYPES_HPP
#define USERHANDLETYPES_HPP
//#include "GameObjectTypes.hpp"
//#include "GameObject.hpp"
//#include "Component.hpp"
//#include "ComponentManager.hpp"
//#include "World.hpp"
namespace Mona {
	template <typename ComponentType>
	class ComponentHandle {
		static_assert(is_component<ComponentType>, "Cannot create handle of type that isn�t a component");
	public:
		static constexpr uint8_t componentIndex = ComponentType::componentIndex;
		ComponentHandle() : m_innerHandle(), m_managerPointer(nullptr) {}
		ComponentHandle(InnerComponentHandle handle, ComponentManager<ComponentType>* manager) : m_innerHandle(handle), m_managerPointer(manager) {}
		InnerComponentHandle GetInnerHandle() const noexcept { return m_innerHandle; }
		bool IsValid() const noexcept {
			if (m_managerPointer == nullptr)
				return false;
			return m_managerPointer->IsValid(m_innerHandle);
		}
		ComponentType* operator->()
		{
			MONA_ASSERT(IsValid(), "ComponentHandle Error: Trying to access with invalid componentHandle");
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
		const ComponentType* operator->() const {
			MONA_ASSERT(IsValid(), "ComponentHandle Error: Trying to access with invalid componentHandle");
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
	private:
		InnerComponentHandle m_innerHandle;
		ComponentManager<ComponentType>* m_managerPointer;
	};
	using TransformHandle = ComponentHandle<TransformComponent>;
	using StaticMeshHandle = ComponentHandle<StaticMeshComponent>;
	using CameraHandle = ComponentHandle<CameraComponent>;
	using RigidBodyHandle = ComponentHandle<RigidBodyComponent>;

	class BaseGameObjectHandle {
	public:
		friend class World;
		BaseGameObjectHandle() : m_innerHandle(), m_objectPointer(nullptr) {}
		BaseGameObjectHandle(InnerGameObjectHandle handle, GameObject* object) : m_innerHandle(handle), m_objectPointer(object) {}
		bool IsValid(World& world) const noexcept {
			return world.IsValid(m_innerHandle);
		}
		const GameObject* operator->() const {
			return m_objectPointer;
		}
		GameObject* operator->() {
			return m_objectPointer;
		}
		GameObject& operator*() {
			return *m_objectPointer;
		}

		const GameObject& operator*() const {
			return *m_objectPointer;
		}
		InnerGameObjectHandle GetInnerHandle() const { return m_innerHandle; }
	protected:
		GameObject* m_objectPointer;
	private:
		InnerGameObjectHandle m_innerHandle;
	};

	template <typename ObjectType>
	class GameObjectHandle : public BaseGameObjectHandle {
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
	public:
		GameObjectHandle() : BaseGameObjectHandle() {}
		GameObjectHandle(InnerGameObjectHandle handle, ObjectType* object) : BaseGameObjectHandle(handle, object) {}
		const ObjectType* operator->() const {
			return static_cast<ObjectType*>(m_objectPointer);
		}
		ObjectType* operator->() {
			return static_cast<ObjectType*>(m_objectPointer);
		}
		ObjectType& operator*() {
			return static_cast<ObjectType&>(*m_objectPointer);
		}

		const ObjectType& operator*() const {
			return static_cast<ObjectType&>(*m_objectPointer);
		}
	};
}
#endif