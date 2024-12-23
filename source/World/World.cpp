#include "World.hpp"
#include "../Core/Config.hpp"
#include "../Event/Events.hpp"
#include "../DebugDrawing/DebugDrawingSystem.hpp"
#include "../Audio/AudioClipManager.hpp"
#include "../PhysicsCollision/PhysicsCollisionSystem.hpp"
#include "../Rendering/Material.hpp"
#include "../Rendering/MeshManager.hpp"
#include "../Rendering/TextureManager.hpp"
#include "../Animation/SkeletonManager.hpp"
#include "../Animation/AnimationClipManager.hpp"
#include "../Animation/AnimationController.hpp"
#include "../ECS/ECS.hpp"
#include <chrono>
namespace Mona {
	
	World::World(Application& app) : 
		m_objectManager(),
		m_eventManager(), 
		m_window(), 
		m_input(), 
		m_application(app),
		m_shouldClose(false),
		m_physicsCollisionSystem(),
		m_ambientLight(glm::vec3(0.1f))
	{
		auto& config = Config::GetInstance();

		m_componentManagers[TransformComponent::componentIndex].reset(new ComponentManager<TransformComponent>());
		m_componentManagers[CameraComponent::componentIndex].reset(new ComponentManager<CameraComponent>());
		m_componentManagers[StaticMeshComponent::componentIndex].reset(new ComponentManager<StaticMeshComponent>());
		m_componentManagers[RigidBodyComponent::componentIndex].reset(new ComponentManager<RigidBodyComponent>());
		m_componentManagers[AudioSourceComponent::componentIndex].reset(new ComponentManager<AudioSourceComponent>());
		m_componentManagers[DirectionalLightComponent::componentIndex].reset(new ComponentManager<DirectionalLightComponent>());
		m_componentManagers[SpotLightComponent::componentIndex].reset(new ComponentManager<SpotLightComponent>());
		m_componentManagers[PointLightComponent::componentIndex].reset(new ComponentManager<PointLightComponent>());
		m_componentManagers[SkeletalMeshComponent::componentIndex].reset(new ComponentManager<SkeletalMeshComponent>());
		m_componentManagers[IKNavigationComponent::componentIndex].reset(new ComponentManager<IKNavigationComponent>());
		m_debugDrawingSystemPhysics.reset(new DebugDrawingSystem_physics());
		m_debugDrawingSystemIKNav.reset(new DebugDrawingSystem_ikNav());
		
		auto& transformDataManager = GetComponentManager<TransformComponent>();
		auto& rigidBodyDataManager = GetComponentManager<RigidBodyComponent>();
		auto& audioSourceDataManager = GetComponentManager<AudioSourceComponent>();
		auto& skeletalMeshDataManager = GetComponentManager<SkeletalMeshComponent>();
		auto& staticMeshDataManager = GetComponentManager<StaticMeshComponent>();
		auto& ikNavigationDataManager = GetComponentManager<IKNavigationComponent>();

		const GameObjectID expectedObjects = config.getValueOrDefault<int>("expected_number_of_gameobjects", 1000);
		rigidBodyDataManager.SetLifetimePolicy(RigidBodyLifetimePolicy(&transformDataManager, &m_physicsCollisionSystem));
		audioSourceDataManager.SetLifetimePolicy(AudioSourceComponentLifetimePolicy(&m_audioSystem));
		ikNavigationDataManager.SetLifetimePolicy(IKNavigationLifetimePolicy(&transformDataManager, 
			&skeletalMeshDataManager,&ikNavigationDataManager));
		m_window.StartUp(m_eventManager);
		m_input.StartUp(m_eventManager);
		m_objectManager.StartUp(expectedObjects);
		for (auto& componentManager : m_componentManagers)
			componentManager->StartUp(m_eventManager, expectedObjects);
		m_application = std::move(app);
		m_renderer.StartUp(m_eventManager, m_debugDrawingSystemIKNav.get());
		//m_renderer.StartUp(m_eventManager, m_debugDrawingSystemPhysics.get());
		m_audioSystem.StartUp();
		m_debugDrawingSystemIKNav->StartUp(&m_ikNavigationSystyem);
		//m_debugDrawingSystemPhysics->StartUp(&m_physicsCollisionSystem);
		m_application.StartUp(*this);
	
	}
	
	World::~World() {
		m_application.UserShutDown(*this);
		m_objectManager.ShutDown(*this);
		for (auto& componentManager : m_componentManagers)
			componentManager->ShutDown(m_eventManager);
		m_audioSystem.ClearSources();
		AudioClipManager::GetInstance().ShutDown();
		m_audioSystem.ShutDown();
		m_physicsCollisionSystem.ShutDown();
		MeshManager::GetInstance().ShutDown();
		TextureManager::GetInstance().ShutDown();
		SkeletonManager::GetInstance().ShutDown();
		AnimationClipManager::GetInstance().ShutDown();
		m_renderer.ShutDown(m_eventManager);
		m_debugDrawingSystemIKNav->ShutDown();
		//m_debugDrawingSystemPhysics->ShutDown();
		m_window.ShutDown();
		m_input.ShutDown(m_eventManager);
		m_eventManager.ShutDown();

	}

	void World::DestroyGameObject(BaseGameObjectHandle& handle) noexcept {
		DestroyGameObject(*handle);
	}

	void World::DestroyGameObject(GameObject& gameObject) noexcept {
		auto& innerComponentHandles = gameObject.m_componentHandles;
		//Es necesario remover primero todas las componentes antes de destruir el GameObject
		for (auto& it : innerComponentHandles) {
			m_componentManagers[it.first]->RemoveComponent(it.second);
		}
		innerComponentHandles.clear();
		m_objectManager.DestroyGameObject(gameObject.GetInnerObjectHandle());
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
		float averageTimeStep = 1.0f/20.0f;
		while (!m_window.ShouldClose() && !m_shouldClose)
		{
			std::chrono::time_point<std::chrono::steady_clock> newTime = std::chrono::steady_clock::now();
			const auto frameTime = newTime - startTime;
			startTime = newTime;
			float timeStep = std::chrono::duration_cast<std::chrono::duration<float>>(frameTime).count();
			float correctedTimeStep = timeStep;
			if (averageTimeStep * 3 < timeStep) {
				correctedTimeStep = averageTimeStep;
			}
			Update(correctedTimeStep);
			if (averageTimeStep*15 < timeStep) {
				timeStep = averageTimeStep;
			}
			averageTimeStep = averageTimeStep * 0.9 + timeStep * 0.1;
		}
		m_eventManager.Publish(ApplicationEndEvent());
		
	}

	void World::Update(float timeStep) noexcept
	{
		auto &transformDataManager = GetComponentManager<TransformComponent>();
		auto &staticMeshDataManager = GetComponentManager<StaticMeshComponent>();
		auto &cameraDataManager = GetComponentManager<CameraComponent>();
		auto& rigidBodyDataManager = GetComponentManager<RigidBodyComponent>();
		auto& audioSourceDataManager = GetComponentManager<AudioSourceComponent>();
		auto& directionalLightDataManager = GetComponentManager<DirectionalLightComponent>();
		auto& spotLightDataManager = GetComponentManager<SpotLightComponent>();
		auto& pointLightDataManager = GetComponentManager<PointLightComponent>();
		auto& skeletalMeshDataManager = GetComponentManager<SkeletalMeshComponent>();
		auto& ikNavigationDataManager = GetComponentManager<IKNavigationComponent>();
		m_input.Update();
		m_physicsCollisionSystem.StepSimulation(timeStep);
		m_physicsCollisionSystem.SubmitCollisionEvents(*this, m_eventManager, rigidBodyDataManager);
		m_ikNavigationSystyem.UpdateAllRigs(ikNavigationDataManager, 
			transformDataManager, 
			staticMeshDataManager, 
			skeletalMeshDataManager, 
			timeStep);
		m_animationSystem.UpdateAllPoses(skeletalMeshDataManager, timeStep);
		m_objectManager.UpdateGameObjects(*this, m_eventManager, timeStep);
		m_application.UserUpdate(*this, timeStep);
		m_audioSystem.Update(m_audoListenerTransformHandle,
			m_audioListenerOffsetRotation,
			timeStep,
			transformDataManager,
			audioSourceDataManager);
		m_renderer.Render(m_eventManager,
			m_cameraHandle,
			m_ambientLight,
			staticMeshDataManager,
			skeletalMeshDataManager,
			transformDataManager,
			cameraDataManager,
			directionalLightDataManager,
			spotLightDataManager,
			pointLightDataManager);
		m_window.Update();
	}

	void World::SetMainCamera(const ComponentHandle<CameraComponent>& cameraHandle) noexcept {
		m_cameraHandle = cameraHandle.GetInnerHandle();

	}

	glm::vec3 World::MainCameraScreenPositionToWorld(const glm::ivec2& screenPos) noexcept
	{
		auto& transformDataManager = GetComponentManager<TransformComponent>();
		auto& cameraDataManager = GetComponentManager<CameraComponent>();
		const CameraComponent* camera = cameraDataManager.GetComponentPointer(m_cameraHandle);
		GameObject* cameraOwner = cameraDataManager.GetOwner(m_cameraHandle);
		TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
		glm::vec3 upVector = cameraTransform->GetUpVector();
		glm::vec3 rightVector = cameraTransform->GetRightVector();
		glm::vec3 frontVector = cameraTransform->GetFrontVector();
		const glm::vec3& cameraPosition = cameraTransform->GetLocalTranslation();
		const glm::ivec2 screenResolution = m_window.GetWindowFrameBufferSize();
		glm::vec2 screenPercentage = glm::vec2((float)screenPos.x / (float)screenResolution.x, (float)screenPos.y / (float)screenResolution.y);
		screenPercentage = glm::vec2(-1.0f) + 2.0f * screenPercentage;
		screenPercentage.y *= -1.0f;
		float cameraHalfYLength = camera->zNearPlane * glm::tan(glm::radians(camera->fieldOfView / 2.0f));
		float cameraHalfXLength = cameraHalfYLength * camera->aspectRatio;
		float upFactor = screenPercentage.y * cameraHalfYLength;
		float rightFactor = screenPercentage.x * cameraHalfXLength;
		float frontFactor = camera->zNearPlane;
		return upFactor * upVector + rightFactor * rightVector + frontFactor * frontVector + cameraPosition;
	}

	ComponentHandle<CameraComponent> World::GetMainCameraComponent() noexcept {
		auto& cameraDataManager = GetComponentManager<CameraComponent>();
		return ComponentHandle<CameraComponent>(m_cameraHandle, &cameraDataManager);
	}
	
	std::shared_ptr<Material> World::CreateMaterial(MaterialType type, bool isForSkinning) noexcept {
		return m_renderer.CreateMaterial(type, isForSkinning);
	}

	void World::SetAudioListenerTransform(const ComponentHandle<TransformComponent>& transformHandle,
		const glm::fquat& offsetRotation) noexcept{
		m_audoListenerTransformHandle = transformHandle.GetInnerHandle();
		m_audioListenerOffsetRotation = offsetRotation;
	}

	ComponentHandle<TransformComponent> World::GetAudioListenerTransform() noexcept {
		auto& transformDataManager = GetComponentManager<TransformComponent>();
		return ComponentHandle<TransformComponent>(m_audoListenerTransformHandle, &transformDataManager);
	}
	void World::SetGravity(const glm::vec3& gravity) {
		m_physicsCollisionSystem.SetGravity(gravity);
	}

	glm::vec3 World::GetGravity() const {
		return m_physicsCollisionSystem.GetGravity();
	}

	ClosestHitRaycastResult World::ClosestHitRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo) {
		auto& rigidBodyDataManager = GetComponentManager<RigidBodyComponent>();
		return m_physicsCollisionSystem.ClosestHitRayTest(rayFrom, rayTo, rigidBodyDataManager);
	}

	AllHitsRaycastResult World::AllHitsRayTest(const glm::vec3& rayFrom, const glm::vec3& rayTo) {
		auto& rigidBodyDataManager = GetComponentManager<RigidBodyComponent>();
		return m_physicsCollisionSystem.AllHitsRayTest(rayFrom, rayTo, rigidBodyDataManager);
	}

	void World::PlayAudioClip3D(std::shared_ptr<AudioClip> audioClip,
		const glm::vec3& position /* = glm::vec3(0.0f) */,
		float volume /* = 1.0f */,
		float pitch /* = 1.0f */,
		float radius /* = 1000.0f */,
		AudioSourcePriority priority /* = AudioSourcePriority::SoundPriorityMedium */)
	{
		m_audioSystem.PlayAudioClip3D(audioClip, position, volume, pitch, radius, priority);
	}

	void World::PlayAudioClip2D(std::shared_ptr<AudioClip> audioClip,
		float volume /* = 1.0f */,
		float pitch /* = 1.0f */,
		AudioSourcePriority priority /* = AudioSourcePriority::SoundPriorityMedium */)
	{
		m_audioSystem.PlayAudioClip2D(audioClip, volume, pitch, priority);
	}

	float World::GetMasterVolume() const noexcept {
		return m_audioSystem.GetMasterVolume();
	}

	void World::SetMasterVolume(float volume) noexcept {
		m_audioSystem.SetMasterVolume(volume);
	}

	JointPose World::GetJointWorldPose(const ComponentHandle<SkeletalMeshComponent>& skeletalMeshHandle, uint32_t jointIndex) noexcept {
		auto transform = GetSiblingComponentHandle<TransformComponent>(skeletalMeshHandle);
		JointPose worldPose(transform->GetLocalRotation(), transform->GetLocalTranslation(), transform->GetLocalScale());
		const AnimationController& animController = skeletalMeshHandle->GetAnimationController();
		return worldPose * animController.GetJointModelPose(jointIndex);
	}

	void World::SetBackgroundColor(float r, float g, float b, float alpha) {
		m_renderer.SetBackgroundColor(r, g, b, alpha);
	}

	void World::EnableECS() {
		m_ecsHandler = std::make_unique<MonaECS::ECSHandler>(this);
	}

	MonaECS::ECSHandler* World::GetECSHandler() {
		if (!m_ecsHandler) {
			EnableECS();
		}
		return m_ecsHandler.get();
	}

}

