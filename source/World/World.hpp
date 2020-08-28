#pragma once
#ifndef WORLD_HPP
#define WORLD_HPP
#include "GameObjectTypes.hpp"
#include "GameObject.hpp"
#include "GameObjectManager.hpp"
#include "Component.hpp"
#include "ComponentManager.hpp"
#include "../Event/EventManager.hpp"
#include "../Platform/Window.hpp"
#include "../Platform/Input.hpp"
#include "../Application.hpp"
#include "../Rendering/CameraComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../Rendering/Renderer.hpp"
#include <memory>
#include <array>
#include <string>

namespace Mona {


	class BaseGameObjectHandle;
	template <typename ObjectType>
	class GameObjectHandle;
	template <typename ComponentType>
	class ComponentHandle;
	

	class World {
	public:
		World();
		World(const World& world) = delete;
		World& operator=(const World& world) = delete;
		
		GameObjectManager::size_type GetGameObjectCount() const noexcept;
		bool IsValid(const InnerGameObjectHandle& handle) const noexcept;
		template <typename ObjectType, typename ...Args>
		GameObjectHandle<ObjectType> CreateGameObject(Args&& ... args) noexcept;
		void DestroyGameObject(BaseGameObjectHandle& handle) noexcept;
		void DestroyGameObject(GameObject& gameObject) noexcept;

		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(BaseGameObjectHandle& objectHandle) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> AddComponent(GameObject& gameObject) noexcept;
		template <typename ComponentType>
		void RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(const BaseGameObjectHandle& objectHandle) const noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(const GameObject& gameObject) const noexcept;
		template <typename ComponentType>
		BaseComponentManager::size_type GetComponentCount() const noexcept;

		EventManager& GetEventManager() noexcept;
		const Input& GetInput() const noexcept;
		Window& GetWindow() noexcept;
		void EndApplication() noexcept;

		void StartUp(std::unique_ptr<Application> app) noexcept;
		void ShutDown() noexcept;
		void StartMainLoop() noexcept;
		void Update(float timeStep) noexcept;

	private:
		template <typename ComponentType>
		auto& GetComponentManager() noexcept;

		EventManager m_eventManager;
		Input m_input;
		Window m_window;
		std::unique_ptr<Application> m_application;
		GameObjectManager m_objectManager;
		std::array<std::unique_ptr<BaseComponentManager>, GetComponentTypeCount()> m_componentManagers;
		Renderer m_renderer;
		bool m_shouldClose;
	};

	

}
#include "Detail/World_Implementation.hpp"
#endif