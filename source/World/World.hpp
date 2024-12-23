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
#include "../Rendering/DirectionalLightComponent.hpp"
#include "../Rendering/PointLightComponent.hpp"
#include "../Rendering/SpotLightComponent.hpp"
#include "../Rendering/Renderer.hpp"
#include "../PhysicsCollision/RigidBodyComponent.hpp"
#include "../PhysicsCollision/RigidBodyLifetimePolicy.hpp"
#include "../PhysicsCollision/RaycastResults.hpp"
#include "../Audio/AudioSystem.hpp"
#include "../Audio/AudioSourceComponent.hpp"
#include "../Audio/AudioSourceComponentLifetimePolicy.hpp"
#include "../Animation/AnimationSystem.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"
#include "../Animation/JointPose.hpp"
#include "../CharacterNavigation/IKNavigationSystem.hpp"
#include "../CharacterNavigation/IKNavigationComponent.hpp"
#include "../CharacterNavigation/IKNavigationLifetimePolicy.hpp"

#include <memory>
#include <array>
#include <filesystem>
#include <string>

namespace MonaECS
{
	class ECSHandler;
}

namespace Mona {

	class Material;
	class World {
	public:
		friend class Engine;
		friend class MonaTest;
		
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

		void SetMainCamera(const ComponentHandle<CameraComponent>& cameraHandle) noexcept;
		glm::vec3 MainCameraScreenPositionToWorld(const glm::ivec2& screenPos) noexcept;
		const glm::vec3& GetAmbientLight() const { return m_ambientLight; }
		void SetAmbientLight(const glm::vec3& light) { m_ambientLight = light; }
		ComponentHandle<CameraComponent> GetMainCameraComponent() noexcept;
		std::shared_ptr<Material> CreateMaterial(MaterialType type, bool isForSkinning = false) noexcept;


		void SetGravity(const glm::vec3& gravity);
		glm::vec3 GetGravity() const;
		ClosestHitRaycastResult ClosestHitRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo);
		AllHitsRaycastResult AllHitsRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo);


		void SetAudioListenerTransform(const ComponentHandle<TransformComponent>& transformHandle,
			const glm::fquat& offRotation = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f)) noexcept;
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

		JointPose GetJointWorldPose(const ComponentHandle<SkeletalMeshComponent>& skeletalMeshHandel, uint32_t jointIndex) noexcept;

		void SetBackgroundColor(float r, float g, float b, float alpha = 0.0f);

		void EnableECS();
		MonaECS::ECSHandler* GetECSHandler();

	private:
		World(Application& app);
		~World();
		void StartMainLoop() noexcept;
		void Update(float timeStep) noexcept;

		template <typename ComponentType>
		auto& GetComponentManager() noexcept;

		template <typename ComponentType, typename ...ComponentTypes>
		bool CheckDependencies(const GameObject& gameObject, DependencyList<ComponentTypes...> dl) const;

		EventManager m_eventManager;
		Input m_input;
		Window m_window;
		Application& m_application;
		bool m_shouldClose;

		GameObjectManager m_objectManager;
		std::array<std::unique_ptr<BaseComponentManager>, GetComponentTypeCount()> m_componentManagers;

		Renderer m_renderer;
		InnerComponentHandle m_cameraHandle;
		glm::vec3 m_ambientLight;

		PhysicsCollisionSystem m_physicsCollisionSystem;
		
		AudioSystem m_audioSystem;
		InnerComponentHandle m_audoListenerTransformHandle;
		glm::fquat m_audioListenerOffsetRotation = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f);

		AnimationSystem m_animationSystem;
		std::unique_ptr<DebugDrawingSystem_physics> m_debugDrawingSystemPhysics;
		std::unique_ptr<DebugDrawingSystem_ikNav> m_debugDrawingSystemIKNav;

		IKNavigationSystem m_ikNavigationSystyem;

		std::unique_ptr<MonaECS::ECSHandler> m_ecsHandler;
	};

	

}
#include "Detail/World_Implementation.hpp"
#endif