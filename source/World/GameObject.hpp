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
		enum class State {
			UnStarted,
			Started,
			PendingDestroy
		};
		GameObject() : m_objectHandle(), m_state(State::UnStarted) {}
		virtual ~GameObject() {};
		GameObject(const GameObject&) = delete;
		GameObject& operator=(const GameObject&) = delete;
		GameObject(GameObject&&) = default;
		GameObject& operator=(GameObject&&) = default;
		
		void StartUp(World& world) noexcept 
		{ 
			UserStartUp(world);
			m_state = State::Started;
		};

		void ShutDown(World& world) noexcept {
			m_state = State::PendingDestroy;
			UserShutDown(world);
		}
		virtual void UserUpdate(World& world, float timeStep) noexcept {};
		virtual void UserStartUp(World& world) noexcept {};
		virtual void UserShutDown(World& world) noexcept {};

		const State GetState() const { return m_state; }
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
		State m_state;
		std::unordered_map<decltype(GetComponentTypeCount()), InnerComponentHandle> m_componentHandles;
	};
}
#endif