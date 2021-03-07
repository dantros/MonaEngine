#pragma once
#ifndef GAMEOBJECT_HPP
#define GAMEOBJECT_HPP
#include <limits>
#include <unordered_map>
#include "GameObjectTypes.hpp"
#include "ComponentTypes.hpp"
namespace Mona {
	class World;
	class GameObjectManager;
	class GameObject {
	public:
		enum class EState {
			Active,
			PendingDestroy
		};
		
		virtual ~GameObject() {};
		
		
		void StartUp(World& world) noexcept 
		{ 
			UserStartUp(world);
		};

		void Update(World& world, float timeStep) noexcept {
			if (m_state == EState::PendingDestroy)
				return;
			UserUpdate(world, timeStep);
		}

		virtual void UserUpdate(World& world, float timeStep) noexcept {};
		virtual void UserStartUp(World& world) noexcept {};

		const EState GetState() const { return m_state; }
		template <typename ComponentType>
		bool HasComponent() const {
			static_assert(is_component<ComponentType>, "Template parameter is not a component");
			auto it = m_componentHandles.find(ComponentType::componentIndex);
			return it != m_componentHandles.end();
		}
		InnerGameObjectHandle GetInnerObjectHandle() const noexcept { return m_objectHandle; }
		template <typename ComponentType>
		InnerComponentHandle GetInnerComponentHandle() const {
			static_assert(is_component<ComponentType>, "Template parameter is not a component");
			auto it = m_componentHandles.find(ComponentType::componentIndex);
			if (it != m_componentHandles.end())
				return it->second;
			else return InnerComponentHandle();
		}
	protected:
		GameObject(const GameObject&) = delete;
		GameObject& operator=(const GameObject&) = delete;
		GameObject(GameObject&&) = default;
		GameObject& operator=(GameObject&&) = default;
		GameObject() : m_objectHandle(), m_state(EState::Active) {}
	private:
		
		friend class GameObjectManager;
		friend class World;
		void ShutDown() noexcept {
			m_state = EState::PendingDestroy;
		}
		void SetObjectHandle(const InnerGameObjectHandle& handle) {
			m_objectHandle = handle;
		}

		void RemoveInnerComponentHandle(decltype(GetComponentTypeCount()) componentIndex){
			m_componentHandles.erase(componentIndex);
		}

		void AddInnerComponentHandle(decltype(GetComponentTypeCount()) componentIndex, InnerComponentHandle componentHandle) {
			m_componentHandles[componentIndex] = componentHandle;
		}
		InnerGameObjectHandle m_objectHandle;
		EState m_state;
		std::unordered_map<decltype(GetComponentTypeCount()), InnerComponentHandle> m_componentHandles;
	};
}
#endif