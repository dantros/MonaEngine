#pragma once
#ifndef WORLD_HPP
#define WORLD_HPP
#include "GameObjectTypes.hpp"
#include "GameObject.hpp"
#include "GameObjectManager.hpp"
#include "ComponentTypes.hpp"
#include "TransformComponent.hpp"
#include "ComponentManager.hpp"
#include "ComponentHandle.hpp"
#include "GameObjectHandle.hpp"
#include "../Event/EventManager.hpp"
#include "../Platform/Window.hpp"
#include "../Platform/Input.hpp"
#include "../Application.hpp"
#include "../Rendering/CameraComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../Rendering/Renderer.hpp"
#include "../PhysicsCollision/RigidBodyComponent.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../PhysicsCollision/RaycastResults.hpp"
#include <memory>
#include <array>
#include <string>

namespace Mona {


	class World {
	public:
		World();
		World(const World& world) = delete;
		World& operator=(const World& world) = delete;
		
		GameObjectManager::size_type GetGameObjectCount() const noexcept;
		bool IsValid(const BaseGameObjectHandle& handle) const noexcept;
		template <typename ObjectType, typename ...Args>
		GameObjectHandle<ObjectType> CreateGameObject(Args&& ... args) noexcept;
		void DestroyGameObject(BaseGameObjectHandle& handle) noexcept;
		void DestroyGameObject(GameObject& gameObject) noexcept;

		template <typename ComponentType, typename ...Args>
		ComponentHandle<ComponentType> AddComponent(BaseGameObjectHandle& objectHandle, Args&& ... args) noexcept;
		template <typename ComponentType, typename ...Args>
		ComponentHandle<ComponentType> AddComponent(GameObject& gameObject, Args&& ... args) noexcept;
		template <typename ComponentType>
		void RemoveComponent(const ComponentHandle<ComponentType>& handle) noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(const BaseGameObjectHandle& objectHandle) const noexcept;
		template <typename ComponentType>
		ComponentHandle<ComponentType> GetComponentHandle(const GameObject& gameObject) const noexcept;
		template <typename SiblingType, typename ComponentType>
		ComponentHandle<SiblingType> GetSiblingComponentHandle(const ComponentHandle<ComponentType>& handle) noexcept;
		template <typename ComponentType>
		BaseComponentManager::size_type GetComponentCount() const noexcept;
		template <typename ComponentType>
		BaseGameObjectHandle GetOwner(const ComponentHandle<ComponentType>& handle) noexcept;

		EventManager& GetEventManager() noexcept;
		Input& GetInput() noexcept;
		Window& GetWindow() noexcept;
		void EndApplication() noexcept;

		void StartUp(std::unique_ptr<Application> app) noexcept;
		void ShutDown() noexcept;
		void StartMainLoop() noexcept;
		void Update(float timeStep) noexcept;

		void SetMainCamera(const BaseGameObjectHandle& objectHandle) noexcept;
		void SetMainCamera(const GameObject& gameObject) noexcept;

		void SetGravity(const glm::vec3& gravity);
		glm::vec3 GetGravity() const;
		ClosestHitRaycastResult ClosestHitRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo);
		AllHitsRaycastResult AllHitsRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo);

	private:
		template <typename ComponentType>
		auto& GetComponentManager() noexcept;

		template <typename ComponentType, typename ...ComponentTypes>
		bool CheckDependencies(const GameObject& gameObject, DependencyList<ComponentTypes...> dl) const;

		EventManager m_eventManager;
		Input m_input;
		Window m_window;
		std::unique_ptr<Application> m_application;
		bool m_shouldClose;

		GameObjectManager m_objectManager;
		std::array<std::unique_ptr<BaseComponentManager>, GetComponentTypeCount()> m_componentManagers;

		Renderer m_renderer;
		InnerComponentHandle m_cameraHandle;

		PhysicsCollisionSystem m_physicsCollisionSystem;

		std::unique_ptr<DebugDrawingSystem> m_debugDrawingSystem;
		
	};

	

}
#include "Detail/World_Implementation.hpp"
#endif