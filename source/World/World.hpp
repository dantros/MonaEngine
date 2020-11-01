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
#include "../Rendering/MeshManager.hpp"
#include "../PhysicsCollision/RigidBodyComponent.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../PhysicsCollision/RaycastResults.hpp"
#include "../Audio/AudioSystem.hpp"
#include "../Audio/AudioClipManager.hpp"
#include "../Audio/AudioSourceComponent.hpp"
#include "../Audio/AudioSourceComponentLifetimePolicy.hpp"
#include <memory>
#include <array>
#include <filesystem>
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

		void SetMainCamera(const ComponentHandle<CameraComponent>& cameraHandle) noexcept;
		ComponentHandle<CameraComponent> GetMainCameraComponent() noexcept;
		std::shared_ptr<Mesh> LoadMesh(MeshManager::PrimitiveType type) noexcept;
		std::shared_ptr<Mesh> LoadMesh(const std::filesystem::path& filePath) noexcept;

		void SetGravity(const glm::vec3& gravity);
		glm::vec3 GetGravity() const;
		ClosestHitRaycastResult ClosestHitRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo);
		AllHitsRaycastResult AllHitsRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo);

		std::shared_ptr<AudioClip> LoadAudioClip(const std::filesystem::path& filePath) noexcept;
		void SetAudioListenerTransform(const ComponentHandle<TransformComponent>& transformHandle) noexcept;
		ComponentHandle<TransformComponent> GetAudioListenerTransform() noexcept;
		void PlayAudioClip3D(std::shared_ptr<AudioClip> audioClip,
			const glm::vec3& position = glm::vec3(0.0f),
			float volume = 1.0f,
			float pitch = 1.0f,
			float radius = 1000.0f,
			AudioSourcePriority priority = AudioSourcePriority::SoundPriorityMedium
		);
		void PlayAudioClip2D(std::shared_ptr<AudioClip> audioClip,
			float volume = 1.0f,
			float pitch = 1.0f,
			AudioSourcePriority priority = AudioSourcePriority::SoundPriorityMedium);
		float GetMasterVolume() const noexcept;
		void SetMasterVolume(float volume) noexcept;

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
		MeshManager m_meshManager;
		InnerComponentHandle m_cameraHandle;

		PhysicsCollisionSystem m_physicsCollisionSystem;
		
		AudioSystem m_audioSystem;
		AudioClipManager m_audioClipManager;
		InnerComponentHandle m_audoListenerTransformHandle;

		std::unique_ptr<DebugDrawingSystem> m_debugDrawingSystem;

		
	};

	

}
#include "Detail/World_Implementation.hpp"
#endif