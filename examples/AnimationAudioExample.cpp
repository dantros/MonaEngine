#include "MonaEngine.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/DiffuseTexturedMaterial.hpp"
#include "Rendering/PBRTexturedMaterial.hpp"
#include <imgui.h>
Mona::CameraHandle CreateCamera(Mona::World& world) {
	auto camera = world.CreateGameObject<Mona::GameObject>();
	auto cameraTransform = world.AddComponent<Mona::TransformComponent>(camera);
	cameraTransform->SetTranslation(glm::vec3(0.0f, -12.0f,  7.0f));
	cameraTransform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.7f);
	auto cameraComp = world.AddComponent<Mona::CameraComponent>(camera);
	world.SetMainCamera(cameraComp);
	return cameraComp;
}

void CreatePlane(Mona::World& world) {
	auto plane = world.CreateGameObject<Mona::GameObject>();
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto materialPtr = std::static_pointer_cast<Mona::UnlitFlatMaterial>(world.CreateMaterial(Mona::MaterialType::UnlitFlat));
	materialPtr->SetColor(glm::vec3(0.3, 0.5f, 0.7f));
	float planeScale = 10.0f;
	auto transform = world.AddComponent<Mona::TransformComponent>(plane);
	transform->SetScale(glm::vec3(planeScale));
	world.AddComponent<Mona::StaticMeshComponent>(plane, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Plane), materialPtr);
	Mona::BoxShapeInformation boxInfo(glm::vec3(planeScale, planeScale, planeScale));
	Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(plane, boxInfo, Mona::RigidBodyType::StaticBody, 1.0f, false, glm::vec3(0.0f,0.0f, -planeScale));
}

void AddDirectionalLight(Mona::World& world, const glm::vec3& axis, float lightIntensity, float angle)
{
	auto light = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(light);
	transform->Rotate(axis, angle);
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}

void AddObjectWithSound(Mona::World &world,
	std::shared_ptr<Mona::Mesh> mesh,
	std::shared_ptr<Mona::Material> material,
	std::shared_ptr<Mona::AudioClip> sound,
	const glm::vec3& pos,
	const glm::fquat& rotation,
	float scale,
	const glm::vec3& boxScale,
	const glm::vec3& colOffset) {

	auto object = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(object);
	transform->SetTranslation(pos);
	transform->SetRotation(rotation);
	transform->Scale(glm::vec3(scale));
	auto& meshManager = Mona::MeshManager::GetInstance();
	world.AddComponent<Mona::StaticMeshComponent>(object, mesh, material);
	auto audioSource = world.AddComponent<Mona::AudioSourceComponent>(object, sound);
	audioSource->SetIsLooping(true);
	audioSource->SetVolume(0.3f);
	audioSource->SetSourceType(Mona::SourceType::Source3D);
	audioSource->SetRadius(7.0f);
	audioSource->Play();
	Mona::BoxShapeInformation boxInfo(boxScale);
	world.AddComponent<Mona::RigidBodyComponent>(object, boxInfo, Mona::RigidBodyType::StaticBody, 1.0f, false, colOffset);
}
class Character : public Mona::GameObject
{
private:
	void SetTargetPosition(const glm::vec3 position) {
		m_targetPosition = position;
		const glm::vec3& currentPos = m_transform->GetLocalTranslation();
		glm::vec3 toTarget = glm::vec3(m_targetPosition.x - currentPos.x,
			m_targetPosition.y - currentPos.y,
			0.0f);
		m_targetFrontVector = glm::normalize(toTarget);
	}

	void UpdateTargetPosition(Mona::World& world) {
		auto& input = world.GetInput();
		if (input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1) && !m_prevIsPress) {
			auto mousePos = input.GetMousePosition();
			MONA_LOG_INFO("Mouse Button press At: ({0},{1}).", mousePos.x, mousePos.y);
			auto camera = world.GetMainCameraComponent();
			auto cameraTransform = world.GetSiblingComponentHandle<Mona::TransformComponent>(camera);
			glm::vec3 rayFrom = cameraTransform->GetLocalTranslation();
			glm::vec3 rayTo = world.MainCameraScreenPositionToWorld(mousePos);
			glm::vec3 direction = glm::normalize(rayTo - rayFrom);
			auto raycastResult = world.ClosestHitRayTest(rayFrom, rayFrom + camera->GetZFarPlane() * direction);
			if (raycastResult.HasHit())
			{
				SetTargetPosition(raycastResult.m_hitPosition);
			}
			m_prevIsPress = true;
		}
		else if (m_prevIsPress && !input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1)) {
			m_prevIsPress = false;
		}

	}


	void UpdateSteeringBehaviour() {
		const glm::vec3& currentPos = m_transform->GetLocalTranslation();
		glm::vec3 toTarget = glm::vec3(m_targetPosition.x - currentPos.x,
			m_targetPosition.y - currentPos.y,
			0.0f);
		float distance = glm::length(toTarget);
		

		
		float speed = distance / m_deacelerationFactor;
		speed = std::min(speed, m_maxSpeed);
		glm::vec3 desiredVelocity = distance >= m_distanceThreshold?  speed * toTarget / distance : glm::vec3(0.0f);
		glm::vec3 rbVelocity = m_rigidBody->GetLinearVelocity();
		glm::vec3 force = glm::vec3(desiredVelocity.x - rbVelocity.x,
			desiredVelocity.y - rbVelocity.y, 0.0f);
		m_rigidBody->SetLinearVelocity(desiredVelocity);
		
		
		glm::vec3 currentFrontVector = m_transform->GetUpVector();
		float cosAngle = glm::dot(currentFrontVector, m_targetFrontVector);
		float angle = glm::acos(cosAngle);
		float sign = glm::cross(currentFrontVector, m_targetFrontVector).z > 0.0f ? 1.0f : -1.0f;
		if (angle != 0.0f) {
			m_rigidBody->SetAngularVelocity(glm::vec3(0.0f, 0.0f, m_angularVelocityFactor*sign * angle));
		}
		


	}

	void UpdateAnimationState() {
		auto& animController = m_skeletalMesh->GetAnimationController();
		glm::vec3 currentVelocity = m_rigidBody->GetLinearVelocity();
		currentVelocity.z = 0.0f;
		float speed = glm::length(currentVelocity);
		
		if (speed > m_maxWalkingSpeed) {
			Mona::BlendType blendType = m_walkingAnimation == animController.GetCurrentAnimation() ? Mona::BlendType::KeepSynchronize : Mona::BlendType::Smooth;
			animController.FadeTo(m_runningAnimation, blendType, m_fadeTime, 0.0f);
		}
		else if (speed > m_maxIdleSpeed) {
			Mona::BlendType blendType = m_runningAnimation == animController.GetCurrentAnimation() ? Mona::BlendType::KeepSynchronize : Mona::BlendType::Smooth;
			animController.FadeTo(m_walkingAnimation, blendType, m_fadeTime, 0.0f);
		}
		else {
			animController.FadeTo(m_idleAnimation, Mona::BlendType::Smooth, m_fadeTime, 0.0f);
		}

	}
public:
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		UpdateTargetPosition(world);
		UpdateSteeringBehaviour();
		UpdateAnimationState();
	};
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_debugGUISubcription, this, &Character::OnDebugGUIEvent);

		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->Rotate(glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(90.f));
		m_transform->SetScale(glm::vec3(0.01f));
		m_transform->Translate(glm::vec3(0.0f,0.0f,01.1f));
		m_targetPosition = glm::vec3(0.0f);
		glm::fquat offsetRotation = glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(180.f), glm::vec3(0.0f, 0.0f, 1.0f)) * glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(-90.f), glm::vec3(1.0f, 0.0f, 0.0f));

		world.SetAudioListenerTransform(m_transform, offsetRotation);
		
		auto materialPtr = std::static_pointer_cast<Mona::DiffuseTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseTextured, true));
		auto& textureManager = Mona::TextureManager::GetInstance();
		auto diffuseTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/akai/akai_diffuse.png"));
		materialPtr->SetMaterialTint(glm::vec3(0.1f));
		materialPtr->SetDiffuseTexture(diffuseTexture);

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton(Mona::SourcePath("Assets/Models/akai_e_espiritu.fbx"));
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, Mona::SourcePath("Assets/Models/akai_e_espiritu.fbx"), true);
		m_runningAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/running.fbx"), skeleton);
		m_walkingAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/walking.fbx"), skeleton);
		m_idleAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/idle.fbx"), skeleton);
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_idleAnimation, materialPtr);
		Mona::BoxShapeInformation boxInfo(glm::vec3(.6f,1.0f,0.6f));
		m_rigidBody = world.AddComponent<Mona::RigidBodyComponent>(*this, boxInfo, Mona::RigidBodyType::DynamicBody, 1.0f, false, glm::vec3(0.0f, -0.1f,0.0f));
		m_rigidBody->ClearForces();

	}

	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Character Options:");
		ImGui::SliderFloat("DeacelerationFactor", &(m_deacelerationFactor), 0.0f, 10.0f);
		ImGui::SliderFloat("DistanceThreshold", &(m_distanceThreshold), 0.0f, 10.0f);
		ImGui::SliderFloat("AngularVelocityFactor", &(m_angularVelocityFactor), 0.0f, 10.0f);
		ImGui::SliderFloat("MaxSpeed", &(m_maxSpeed), 0.0f, 10.0f);
		ImGui::SliderFloat("MaxIdleSpeed", &(m_maxIdleSpeed), 0.0f, 10.0f);
		ImGui::SliderFloat("MaxWalkingSpeed", &(m_maxWalkingSpeed), 0.0f, 10.0f);
		ImGui::SliderFloat("FadeTime", &(m_fadeTime), 0.0f, 1.0f);
		ImGui::End();
	}
private:
	float m_deacelerationFactor = 1.1f;
	float m_angularVelocityFactor = 3.0f;
	float m_distanceThreshold = 0.65f;
	float m_maxSpeed = 5.0f;
	float m_maxIdleSpeed = 0.2f;
	float m_maxWalkingSpeed = 3.4f;
	float m_fadeTime = 0.5f;
	bool m_prevIsPress = false;
	glm::vec3 m_targetPosition = glm::vec3(0.0f);
	glm::vec3 m_targetFrontVector = glm::vec3(0.0f,-1.0f,0.0f);
	Mona::TransformHandle m_transform;
	Mona::RigidBodyHandle m_rigidBody;
	Mona::SkeletalMeshHandle m_skeletalMesh;
	std::shared_ptr<Mona::AnimationClip> m_runningAnimation;
	std::shared_ptr<Mona::AnimationClip> m_walkingAnimation;
	std::shared_ptr<Mona::AnimationClip> m_idleAnimation;
	Mona::SubscriptionHandle m_debugGUISubcription;

};
class AnimationAudio : public Mona::Application
{
public:
	AnimationAudio() = default;
	~AnimationAudio() = default;
	virtual void UserStartUp(Mona::World& world) noexcept override {
		world.SetGravity(glm::vec3(0.0f, 0.0f, 0.0f));
		world.SetAmbientLight(glm::vec3(0.1f));
		CreateCamera(world);
		CreatePlane(world);
		world.CreateGameObject<Character>();
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-45.0f));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-135.0f));

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto ventMesh = meshManager.LoadMesh(Mona::SourcePath("Assets/Models/AirConditionerOBJ/AirConditioner.obj"), true);
		auto boomBoxMesh = meshManager.LoadMesh(Mona::SourcePath("Assets/Models/BoomBoxOBJ/BoomBox.obj"),true);

		auto ventMaterial = std::static_pointer_cast<Mona::PBRTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::PBRTextured));
		auto boomBoxMaterial = std::static_pointer_cast<Mona::PBRTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::PBRTextured));
		auto& textureManager = Mona::TextureManager::GetInstance();
		
		auto ventAoTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/AirConditionerOBJ/AO.png"));
		auto ventAlbedoTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/AirConditionerOBJ/Albedo.png"));
		auto ventMetallicTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/AirConditionerOBJ/Metallic.png"));
		auto ventRoughnessTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/AirConditionerOBJ/Roughness.png"));
		auto ventNormalTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/AirConditionerOBJ/Normal_Map.png"));
		
		ventMaterial->SetAmbientOcclusionTexture(ventAoTexture);
		ventMaterial->SetAlbedoTexture(ventAlbedoTexture);
		ventMaterial->SetMetallicTexture(ventMetallicTexture);
		ventMaterial->SetRoughnessTexture(ventRoughnessTexture);
		ventMaterial->SetNormalMapTexture(ventNormalTexture);

		

		auto boomBoxAoTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BoomBoxOBJ/AO.jpeg"));
		auto boomBoxAlbedoTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BoomBoxOBJ/Albedo.jpeg"));
		auto boomBoxMetallicTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BoomBoxOBJ/Metallic.jpeg"));
		auto boomBoxRoughnessTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BoomBoxOBJ/Roughness.jpeg"));
		auto boomBoxNormalTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BoomBoxOBJ/Normal_Map.jpeg"));
		
		boomBoxMaterial->SetAmbientOcclusionTexture(boomBoxAoTexture);
		boomBoxMaterial->SetAlbedoTexture(boomBoxAlbedoTexture);
		boomBoxMaterial->SetMetallicTexture(boomBoxMetallicTexture);
		boomBoxMaterial->SetRoughnessTexture(boomBoxRoughnessTexture);
		boomBoxMaterial->SetNormalMapTexture(boomBoxNormalTexture);
		
		auto & audioManager = Mona::AudioClipManager::GetInstance();
		auto ventilationSound = audioManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/mono_fan_ventilation.wav"));
		auto musicSound = audioManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/mono_music.wav"));
		
		
		
		
		glm::fquat rotation = glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(90.f), glm::vec3(1.0f, 0.0f, 0.0f));
		AddObjectWithSound(world, ventMesh, ventMaterial, ventilationSound, glm::vec3(-8.0f, 0.0f, 0.5f), rotation, 0.02f, glm::vec3(.8f,.7f,.4f), glm::vec3(0.0f,-0.1f,0.3f));
		rotation = glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(180.f), glm::vec3(0.0f, 0.0f, 1.0f)) * rotation;
		AddObjectWithSound(world, boomBoxMesh, boomBoxMaterial, musicSound, glm::vec3(7.0f, 0.0f, 0.0f), rotation, 1.0f, glm::vec3(1.5f,.8f,.4f), glm::vec3(-0.1f, .8f,0.0f));
		auto &em = world.GetEventManager();
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {

	}

};

int main()
{
	AnimationAudio app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
}