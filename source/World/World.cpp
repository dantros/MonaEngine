#include "World.hpp"
#include "../Core/Config.hpp"
#include "../Event/Events.hpp"
#include "../DebugDrawing/DebugDrawingSystem.hpp"
#include "../PhysicsCollision/PhysicsCollisionSystem.hpp"
#include <chrono>
namespace Mona {
	
	World::World() : 
		m_objectManager(),
		m_eventManager(), 
		m_window(), 
		m_input(), 
		m_application(),
		m_shouldClose(false),
		m_physicsCollisionSystem() {
		
		m_componentManagers[TransformComponent::componentIndex].reset(new ComponentManager<TransformComponent>());
		m_componentManagers[CameraComponent::componentIndex].reset(new ComponentManager<CameraComponent>());
		m_componentManagers[StaticMeshComponent::componentIndex].reset(new ComponentManager<StaticMeshComponent>());
		m_componentManagers[RigidBodyComponent::componentIndex].reset(new ComponentManager<RigidBodyComponent, RigidBodyLifetimePolicy>());
		m_debugDrawingSystem.reset(new DebugDrawingSystem());
	
	}
	void World::StartUp(std::unique_ptr<Application> app) noexcept {
		auto& transformDataManager = GetComponentManager<TransformComponent>();
		auto& rigidBodyDataManager = GetComponentManager<RigidBodyComponent>();
		auto& config = Config::GetInstance();
		const GameObjectID expectedObjects = config.getValueOrDefault<int>("expected_number_of_gameobjects", 1000);

		m_window.StartUp(m_eventManager);
		m_input.StartUp(m_eventManager);
		m_objectManager.StartUp(expectedObjects);
		for (auto& componentManager : m_componentManagers)
			componentManager->StartUp(m_eventManager, expectedObjects);
		m_application = std::move(app);
		m_renderer.StartUp(m_eventManager, m_debugDrawingSystem.get());
		m_physicsCollisionSystem.StartUp(transformDataManager, rigidBodyDataManager);
		m_debugDrawingSystem->StartUp(&m_physicsCollisionSystem);
		m_application->StartUp(*this);
	}

	void World::ShutDown() noexcept {
		m_application->UserShutDown(*this);
		m_objectManager.ShutDown(*this);
		m_physicsCollisionSystem.ShutDown();
		for (auto& componentManager : m_componentManagers)
			componentManager->ShutDown(m_eventManager);
		
		m_renderer.ShutDown(m_eventManager);
		m_window.ShutDown();
		m_input.ShutDown(m_eventManager);
		m_eventManager.ShutDown();

	}

	void World::DestroyGameObject(BaseGameObjectHandle& handle) noexcept {
		DestroyGameObject(*handle);
	}

	void World::DestroyGameObject(GameObject& gameObject) noexcept {
		auto& innerComponentHandles = gameObject.m_componentHandles;
		for (auto& it : innerComponentHandles) {
			m_componentManagers[it.first]->RemoveComponent(it.second);
		}
		innerComponentHandles.clear();
		m_objectManager.DestroyGameObject(*this, gameObject.GetInnerObjectHandle());
	}

	bool World::IsValid(const BaseGameObjectHandle& handle) const noexcept {
		return m_objectManager.IsValid(handle->GetInnerObjectHandle());
	}
	GameObjectManager::size_type World::GetGameObjectCount() const noexcept
	{
		return m_objectManager.GetCount();
	}
	EventManager& World::GetEventManager() noexcept {
		return m_eventManager;
	}

	Input& World::GetInput() noexcept {
		return m_input;
	}

	Window& World::GetWindow() noexcept {
		return m_window;
	}

	void World::EndApplication() noexcept {
		m_shouldClose = true;
	}

	void World::StartMainLoop() noexcept {
		std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
		
		while (!m_window.ShouldClose() && !m_shouldClose)
		{
			std::chrono::time_point<std::chrono::steady_clock> newTime = std::chrono::steady_clock::now();
			const auto frameTime = newTime - startTime;
			startTime = newTime;
			float timeStep = std::chrono::duration_cast<std::chrono::duration<float>>(frameTime).count();
			float ms = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(frameTime).count();
			Update(timeStep);
		}
		m_eventManager.Publish(ApplicationEndEvent());
		
	}

	void World::Update(float timeStep) noexcept
	{
		auto &transformDataManager = GetComponentManager<TransformComponent>();
		auto &staticMeshDataManager = GetComponentManager<StaticMeshComponent>();
		auto &cameraDataManager = GetComponentManager<CameraComponent>();
		auto& rigidBodyDataManager = GetComponentManager<RigidBodyComponent>();
		m_input.Update();
		m_physicsCollisionSystem.StepSimulation(timeStep);
		m_physicsCollisionSystem.SubmitCollisionEvents(rigidBodyDataManager);
		m_objectManager.UpdateGameObjects(*this, timeStep);
		m_application->UserUpdate(*this, timeStep);
		m_renderer.Render(m_eventManager, m_cameraHandle, staticMeshDataManager, transformDataManager, cameraDataManager);
		m_window.Update();
	}

	void World::SetMainCamera(const BaseGameObjectHandle& objectHandle) noexcept {
		SetMainCamera(*objectHandle);
	}

	void World::SetMainCamera(const GameObject& gameObject) noexcept {
		MONA_ASSERT(gameObject.HasComponent<CameraComponent>(), "Given GameObject doesn't have a camera component");
		m_cameraHandle = gameObject.GetInnerComponentHandle<CameraComponent>();

	}

	void World::SetGravity(const glm::vec3& gravity) {
		m_physicsCollisionSystem.SetGravity(gravity);
	}

	glm::vec3 World::GetGravity() const {
		return m_physicsCollisionSystem.GetGravity();
	}
}

