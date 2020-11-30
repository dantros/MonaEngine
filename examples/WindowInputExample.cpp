#include "MonaEngine.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "Rendering/PBRTexturedMaterial.hpp"
#include <imgui.h>
class Box : public Mona::GameObject {
public:
	Box(float speed, float rspeed) {
		m_speed = speed;
		m_rotationSpeed = rspeed;
	}
	void UserStartUp(Mona::World& world) noexcept override {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->Scale(glm::vec3(1.0f / 20000.0f));
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
		std::shared_ptr<Mona::PBRTexturedMaterial> material = std::static_pointer_cast<Mona::PBRTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::PBRTextured));
		std::shared_ptr<Mona::Texture> albedo = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_albedo.jpg"));
		std::shared_ptr<Mona::Texture> normalMap = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_normal.jpg"));
		std::shared_ptr<Mona::Texture> metallic = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_metallic.jpg"));
		std::shared_ptr<Mona::Texture> roughness = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_roughness.jpg"));
		std::shared_ptr<Mona::Texture> ambientOcclusion = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/DrakePistolOBJ/base_AO.jpg"));
		material->SetAlbedoTexture(albedo);
		material->SetNormalMapTexture(normalMap);
		material->SetMetallicTexture(metallic);
		material->SetRoughnessTexture(roughness);
		material->SetAmbientOcclusionTexture(ambientOcclusion);
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, model, material);


	}
	void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		m_transform->Translate(glm::vec3(m_speed, 0.0f, m_speed) * timeStep);
		m_transform->Rotate(glm::vec3(0.0f, 0.0f, 1.0f), m_rotationSpeed * timeStep);
	}
	float m_rotationSpeed;
private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_staticMesh;
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
		world.SetAmbientLight(glm::vec3(0.03f));
		auto& eventManager = world.GetEventManager();
		m_windowResizeSubcription = eventManager.Subscribe(this, &Sandbox::OnWindowResize);
		m_debugGUISubcription = eventManager.Subscribe(this, &Sandbox::OnDebugGUIEvent);
		world.CreateGameObject<Sphere>(0.0f, 0.0f);
		m_rotatingBox = world.CreateGameObject<Box>(0.0f, 0.0f);
		auto camera = world.CreateGameObject<Mona::BasicPerspectiveCamera>();
		world.AddComponent<Mona::SpotLightComponent>(camera, glm::vec3(10.0f), 15.0f, glm::radians(25.0f), glm::radians(37.0f));
		
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(camera));
		world.GetInput().SetCursorType(Mona::Input::CursorType::Disabled);

		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-45.0f));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), 10.0f, glm::radians(-135.0f));

		/*
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& textureManager = Mona::TextureManager::GetInstance();
		std::shared_ptr<Mona::Mesh> planeModel = meshManager.LoadMesh(Mona::MeshManager::PrimitiveType::Plane);
		std::shared_ptr<Mona::PBRTexturedMaterial> planeMaterial = std::static_pointer_cast<Mona::PBRTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::PBRTextured));
		std::shared_ptr<Mona::Texture> planeTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Textures/brickwall.jpg"));
		std::shared_ptr<Mona::Texture> planeNormalMap = textureManager.LoadTexture(Mona::SourcePath("Assets/Textures/brickwall_normal.jpg"));
		planeMaterial->SetDiffuseTexture(planeTexture);
		planeMaterial->SetNormalMapTexture(planeNormalMap);
		world.AddComponent<Mona::StaticMeshComponent>(plane, planeModel, planeMaterial);*/
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: Sandbox");
		auto& eventManager = world.GetEventManager();
		eventManager.Unsubscribe(m_debugGUISubcription);
		eventManager.Unsubscribe(m_windowResizeSubcription);
	}
	
	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Testing Float Slider:");
		ImGui::SliderFloat("BoxRotationSpeed", &(m_rotatingBox->m_rotationSpeed), 0.0f, 10.0f);
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
		else if (input.IsKeyPressed(MONA_KEY_D)) {
		}
		else if (input.GetMouseWheelOffset().y > 0.0)
		{
			auto Offset = input.GetMouseWheelOffset();
			MONA_LOG_INFO("The mouse offset is ({0},{1})", Offset.x, Offset.y);

		}
	}
private:
	Mona::SubscriptionHandle m_windowResizeSubcription;
	Mona::SubscriptionHandle m_debugGUISubcription;
	Mona::GameObjectHandle<Box> m_rotatingBox;
	float somefloat = 0.0f;
};
int main()
{	
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Sandbox()));
	engine.StartMainLoop();
	engine.ShutDown();
}