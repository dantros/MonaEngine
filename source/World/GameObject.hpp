#pragma once
#ifndef GAMEOBJECT_HPP
#define GAMEOBJECT_HPP
#include "../Core/Log.hpp"
#include <limits>
#include <unordered_map>
#include "GameObjectTypes.hpp"
#include "Component.hpp"
namespace Mona {
	class World;
	class GameObjectManager;
	class GameObject {
	public:
		GameObject() : m_objectHandle() {}
		GameObject(const GameObject&) = delete;
		GameObject& operator=(const GameObject&) = delete;
		GameObject(GameObject&&) = default;
		GameObject& operator=(GameObject&&) = default;
		virtual void Update(World& world, float timeStep) noexcept {};
		virtual void StartUp(World& world) noexcept {};
		virtual void ShutDown(World& world) noexcept {};
		virtual ~GameObject() {
		//TODO(BYRON): Add removal of components;
		};
		InnerGameObjectHandle GetObjectHandle() const noexcept{ return m_objectHandle; }
		template <typename ComponentType>
		InnerComponentHandle GetInnerComponentHandle() const {
			auto& it = m_componentHandles.find(ComponentType::componentIndex);
			if (it != m_componentHandles.end())
				return it->second;
			else return InnerComponentHandle();
		}
	private:
		friend class GameObjectManager;
		friend class World;
		void SetObjectHandle(const InnerGameObjectHandle& handle) {
			m_objectHandle = handle;
		}
		InnerGameObjectHandle m_objectHandle;
		std::unordered_map<decltype(GetComponentTypeCount()), InnerComponentHandle> m_componentHandles;
	};
}
#endif