#pragma once
#ifndef COMPONENTHANDLE_HPP
#define COMPONENTHANDLE_HPP
#include "../Core/Log.hpp"
#include <memory>

namespace Mona {
	template <typename ComponentType>
	class ComponentManager;

	template <typename ComponentType>
	class ComponentHandle
	{
	public:
		ComponentHandle() : m_managerPtr(nullptr), m_ownerID(INVALID_INDEX) {};
		ComponentHandle(ComponentManager<ComponentType>* managerPtr, GameObjectID ownerID)
			: m_managerPtr(managerPtr), m_ownerID(ownerID) {}
		GameObjectID GetOwnerID() const noexcept { return m_ownerID; }
		ComponentType* operator->() {
			auto ptr = m_managerPtr->GetComponentPtr(*this);
			MONA_ASSERT(ptr != nullptr, "WORLD: Trying to dereference Invalid Handle!");
			return ptr;
		}
	private:
		ComponentManager<ComponentType>* m_managerPtr;
		GameObjectID m_ownerID;
	};
}


#endif