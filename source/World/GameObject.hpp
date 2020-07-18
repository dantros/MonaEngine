#pragma once
#ifndef GAMEOBJECT_HPP
#define GAMEOBJECT_HPP
#include "../Core/Log.hpp"
#include <limits>
#include "GameObjectTypes.hpp"
#include "Component.hpp"
#include "ComponentManager.hpp"
namespace Mona {
	class World;
	template <typename ComponentType>
	class UserComponentHandle {
	public:
		UserComponentHandle() : m_innerHandle(), m_managerPointer(nullptr) {}
		UserComponentHandle(ComponentHandle handle, ComponentManager<ComponentType>* managerPointer) :
			m_innerHandle(handle), m_managerPointer(managerPointer) {}
		bool IsValid() const {
			return m_managerPointer!= nullptr && m_managerPointer->IsValid(m_innerHandle);
		}
		
		ComponentType* operator->() {
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
		ComponentHandle GetInnerHandle() const {
			return m_innerHandle;
		}
	private:
		ComponentHandle m_innerHandle;
		ComponentManager<ComponentType>* m_managerPointer;
	};

	class GameObject {
	public:
		GameObject(){
			m_id = CreateNewObjectIndex();
		}
		virtual void Update(World& world, float timeStep) noexcept {};
		virtual void StartUp(World& world) noexcept {};
		GameObjectID GetObjectID() const noexcept{ return m_id; }

	private:
		GameObjectID m_id;
		static GameObjectID CreateNewObjectIndex() {

			static GameObjectID currentIndex = 0;
			MONA_ASSERT(currentIndex < std::numeric_limits<GameObjectID>::max(), "WORLD: Cannot create another GameObject, maximun number of them has been reached");
			return ++currentIndex;
		}

	};
	
	template <typename ComponentType>
	UserComponentHandle<ComponentType> AddComponent(World& world, GameObject& object)
	{
		auto managerPtr = world.GetManagerPointer<ComponentType>();
		return UserComponentHandle<ComponentType>(world.AddComponent<ComponentType>(object.GetObjectID()), managerPtr);
	}

	template <typename ComponentType>
	UserComponentHandle<ComponentType> RemoveComponent(World& world, UserComponentHandle<ComponentType>& userHandle)
	{
		world.RemoveComponent(userHandle.GetInnerHandle());
	}

	using TransformHandle = UserComponentHandle<TransformComponent>;
	using StaticMeshHandle = UserComponentHandle<StaticMeshComponent>;
}
#endif