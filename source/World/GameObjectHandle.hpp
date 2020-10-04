#pragma once
#ifndef GAMEOBJECTHANDLE_HPP
#define GAMEOBJECTHANDLE_HPP
#include "GameObjectTypes.hpp"
#include "GameObject.hpp"
namespace Mona {
	class BaseGameObjectHandle {
	public:
		friend class World;
		BaseGameObjectHandle() : m_innerHandle(), m_objectPointer(nullptr) {}
		BaseGameObjectHandle(InnerGameObjectHandle handle, GameObject* object) : m_innerHandle(handle), m_objectPointer(object) {}
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