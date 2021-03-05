#include "MonaEngine.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "Rendering/PBRTexturedMaterial.hpp"
#include "Rendering/UnlitTexturedMaterial.hpp"
#include "Rendering/DiffuseTexturedMaterial.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include <imgui.h>
class AnimatedMesh : public Mona::GameObject {
public:
	AnimatedMesh() = default;
	void UserStartUp(Mona::World& world) noexcept override {
		
		auto transform = world.AddComponent<Mona::TransformComponent>(*this);
		transform->SetScale(glm::vec3(0.05f));
		transform->Translate(glm::vec3(3.0f));
		std::shared_ptr<Mona::DiffuseFlatMaterial> material = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat, true));
		material->SetDiffuseColor(0.1f * glm::vec3(0.3f, 0.75f, 0.1f));
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton(Mona::SourcePath("Assets/Models/xbot.fbx"));
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, Mona::SourcePath("Assets/Models/xbot.fbx"));
		//auto [newModel, skeleton] = meshManager.LoadMeshWithSkeleton(Mona::SourcePath("Assets/Idle.fbx"));
		m_animation0 = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/running.fbx"), skeleton);
		m_animation1 = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/walking.fbx"), skeleton);
		m_animation2 = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/idle.fbx"), skeleton);

		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_animation0, material);

	}

	void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		auto& input = world.GetInput();
		auto& animController = m_skeletalMesh->GetAnimationController();
		if (input.IsKeyPressed(MONA_KEY_9))
		{
			animController.PlayAnimation(m_animation0);
		}
		else if (input.IsKeyPressed(MONA_KEY_8))
		{
			animController.PlayAnimation(m_animation1);
		}
		else if (input.IsKeyPressed(MONA_KEY_7)) {
			//m_skeletalMesh->SetIsLooping(false);
			//m_skeletalMesh->SetPlayRate(0.0f);
			animController.FadeTo(m_animation1, Mona::BlendType::KeepSynchronize, 0.5f, 0.0f);
		}

	}
private:
	Mona::SkeletalMeshHandle m_skeletalMesh;
	std::shared_ptr<Mona::AnimationClip> m_animation0;
	std::shared_ptr<Mona::AnimationClip> m_animation1;
	std::shared_ptr<Mona::AnimationClip> m_animation2;

};
class Box : public Mona::GameObject {
public:
	Box(float speed, float rspeed) {
		m_speed = speed;
		m_rotationSpeed = rspeed;
	}
	void UserStartUp(Mona::World& world) noexcept override {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->Scale(glm::vec3(1.0f / 200.0f));
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& textureManager = Mona::TextureManager::GetInstance();
		std::shared_ptr<Mona::Mesh> model = meshManager.LoadMesh(Mona::SourcePath("Assets/Models/BackpackFBX/Survival_BackPack_2.fbx"), true);
		std::shared_ptr<Mona::PBRTexturedMaterial> material = std::static_pointer_cast<Mona::PBRTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::PBRTextured));
		std::shared_ptr<Mona::Texture> albedo = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BackpackFBX/1001_albedo.jpg"));
		std::shared_ptr<Mona::Texture> normalMap = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BackpackFBX/1001_normal.png"));
		std::shared_ptr<Mona::Texture> metallic = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BackpackFBX/1001_metallic.jpg"));
		std::shared_ptr<Mona::Texture> roughness = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BackpackFBX/1001_roughness.jpg"));
		std::shared_ptr<Mona::Texture> ambientOcclusion = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/BackpackFBX/1001_AO.jpg"));
		material->SetAlbedoTexture(albedo);
		material->SetNormalMapTexture(normalMap);
		material->SetMetallicTexture(metallic);
		material->SetRoughnessTexture(roughness);
		material->SetAmbientOcclusionTexture(ambientOcclusion);
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, model, material);


	}
	void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		m_transform->Translate(glm::vec3(m_speed, 0.0f, m_speed)*timeStep);
		m_transform->Rotate(glm::vec3(0.0f,0.0f,1.0f), m_rotationSpeed*timeStep);
	}

	float m_rotationSpeed;
private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_staticMesh;
	float m_speed;
	
};

class Sphere : public Mona::GameObject {
public:
	Sphere(float speed, float rspeed) {
		m_speed = speed;
		m_rotationSpeed = rspeed;
	}
	void UserStartUp(Mona::World& world) noexcept override {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->Translate(glm::vec3(0.0f, 0.0f, 4.0f));
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& textureManager = Mona::TextureManager::GetInstance();
		std::shared_ptr<Mona::Mesh> model = meshManager.LoadMesh(Mona::SourcePath("Assets/Models/DrakePistolOBJ/drakefire_pistol_low.obj"),true);
		m_pbrMaterial = std::static_pointer_cast<Mona::PBRTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::PBRTextured));
		m_diffuseMaterial = std::static_pointer_cast<Mona::DiffuseTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseTextured));
		m_unlitMaterial = std::static_pointer_cast<Mona::UnlitTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::UnlitTextured));

		std::shared_ptr<Mona::Texture> albedo = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_albedo.jpg"));
		std::shared_ptr<Mona::Texture> normalMap = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_normal.jpg"));
		std::shared_ptr<Mona::Texture> metallic = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_metallic.jpg"));
		std::shared_ptr<Mona::Texture> roughness = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_roughness.jpg"));
		std::shared_ptr<Mona::Texture> ambientOcclusion = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_AO.jpg"));
		m_pbrMaterial->SetAlbedoTexture(albedo);
		m_pbrMaterial->SetNormalMapTexture(normalMap);
		m_pbrMaterial->SetMetallicTexture(metallic);
		m_pbrMaterial->SetRoughnessTexture(roughness);
		m_pbrMaterial->SetAmbientOcclusionTexture(ambientOcclusion);

		m_diffuseMaterial->SetDiffuseTexture(albedo);
		m_diffuseMaterial->SetMaterialTint(glm::vec3(0.3f));
		m_unlitMaterial->SetUnlitColorTexture(albedo);
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, model, m_pbrMaterial);



	}

	void ChangeMaterial(int index) {
		if (index == 0) {
			m_staticMesh->SetMaterial(m_pbrMaterial);
		}
		else if (index == 1)
		{
			m_staticMesh->SetMaterial(m_diffuseMaterial);
		}
		else if (index == 2)
		{
			m_staticMesh->SetMaterial(m_unlitMaterial);
		}
	}
	void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		m_transform->Translate(glm::vec3(m_speed, 0.0f, m_speed) * timeStep);
		m_transform->Rotate(glm::vec3(0.0f, 0.0f, 1.0f), m_rotationSpeed * timeStep);
	}
	float m_rotationSpeed;
private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_staticMesh;
	std::shared_ptr<Mona::PBRTexturedMaterial> m_pbrMaterial = nullptr;
	std::shared_ptr<Mona::DiffuseTexturedMaterial> m_diffuseMaterial = nullptr;
	std::shared_ptr<Mona::UnlitTexturedMaterial> m_unlitMaterial = nullptr;
	float m_speed;

};

void AddDirectionalLight(Mona::World& world, const glm::vec3& axis, float lightIntensity, float angle)
{
	auto light = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(light);
	transform->Rotate(axis, angle);
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}

class Sandbox : public Mona::Application
{
public:
	Sandbox() = default;
	~Sandbox() = default;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		MONA_LOG_INFO("Starting User App: Sandbox");
		world.SetAmbientLight(glm::vec3(0.0f));
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_windowResizeSubcription, this, &Sandbox::OnWindowResize);
		eventManager.Subscribe(m_debugGUISubcription, this, &Sandbox::OnDebugGUIEvent);
		m_sphere = world.CreateGameObject<Sphere>(0.0f, 0.0f);
		m_rotatingBox = world.CreateGameObject<Box>(0.0f, 0.0f);
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera>();
		world.AddComponent<Mona::SpotLightComponent>(m_camera, glm::vec3(100.0f), 15.0f, glm::radians(25.0f), glm::radians(37.0f));
		world.CreateGameObject<AnimatedMesh>();
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		world.GetInput().SetCursorType(Mona::Input::CursorType::Disabled);

	  AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-45.0f));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-135.0f));

		auto sphere = world.CreateGameObject<Mona::GameObject>();
		auto sphereTransform = world.AddComponent<Mona::TransformComponent>(sphere);
		sphereTransform->Scale(glm::vec3(0.1f));
		sphereTransform->Translate(glm::vec3(3.0f,3.0f,3.0f));
		world.AddComponent<Mona::StaticMeshComponent>(sphere,
			Mona::MeshManager::GetInstance().LoadMesh(Mona::Mesh::PrimitiveType::Cube),
			world.CreateMaterial(Mona::MaterialType::UnlitFlat)
			);
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: Sandbox");
		auto& eventManager = world.GetEventManager();
		eventManager.Unsubscribe(m_debugGUISubcription);
		eventManager.Unsubscribe(m_windowResizeSubcription);
	}
	
	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Scene Options:");
		ImGui::SliderFloat("BagRotationSpeed", &(m_rotatingBox->m_rotationSpeed), 0.0f, 10.0f);
		static bool selected[3] = { false, false, false };
		if (ImGui::RadioButton("PBRMaterial", &m_currentMaterialIndex, 0)) {
			m_sphere->ChangeMaterial(0);
		}
		if (ImGui::RadioButton("DiffuseMaterial", &m_currentMaterialIndex, 1)) {
			m_sphere->ChangeMaterial(1);
		}
		if (ImGui::RadioButton("UnlitMaterial", &m_currentMaterialIndex, 2)) {
			m_sphere->ChangeMaterial(2);
		}
		ImGui::End();
	}

	void OnWindowResize(const Mona::WindowResizeEvent& event)
	{
		MONA_LOG_INFO("A WindowResizeEvent has ocurred! {0} {1}", event.width, event.height);
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		if (input.IsKeyPressed(MONA_KEY_G))
		{
			window.SetFullScreen(true);
		}
		else if (input.IsKeyPressed(MONA_KEY_H))
		{
			window.SetFullScreen(false);
		}
		else if (input.IsKeyPressed(MONA_KEY_J))
		{
			window.SetWindowDimensions(glm::ivec2(1000, 1000));
		}
		else if (input.IsKeyPressed(MONA_KEY_1)) {
			m_camera->SetActive(false);
			input.SetCursorType(Mona::Input::CursorType::Normal);
		}
		else if (input.IsKeyPressed(MONA_KEY_2)) {
			m_camera->SetActive(true);
			input.SetCursorType(Mona::Input::CursorType::Disabled);
		}
	}
private:
	Mona::SubscriptionHandle m_windowResizeSubcription;
	Mona::SubscriptionHandle m_debugGUISubcription;
	Mona::GameObjectHandle<Box> m_rotatingBox;
	Mona::GameObjectHandle<Sphere> m_sphere;
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera> m_camera;
	float somefloat = 0.0f;
	int m_currentMaterialIndex;
	
};
int main()
{	
	Sandbox app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
}