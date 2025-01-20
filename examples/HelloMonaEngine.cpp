#include "MonaEngine.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "Rendering/PBRTexturedMaterial.hpp"
#include "Rendering/UnlitTexturedMaterial.hpp"
#include "Rendering/DiffuseTexturedMaterial.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include <imgui.h>

class Box : public Mona::GameObject {
public:
	Box(float speed, float rotationSpeed) {
		m_speed = speed;
		m_rotationSpeed = rotationSpeed;
	}
	void UserStartUp(Mona::World& world) noexcept override {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->SetTranslation(glm::vec3(0, 0, 0));
		m_transform->SetRotation(glm::angleAxis(glm::radians(90.f), glm::vec3(1, 0, 0)));
		m_transform->SetScale(glm::vec3(4, 4, 4));

		auto& config = Mona::Config::GetInstance();
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& textureManager = Mona::TextureManager::GetInstance();

		auto cubeMesh = meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube);

		auto redMaterial = world.CreateMaterial(Mona::MaterialType::DiffuseFlat);
		auto redMaterialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(redMaterial);
		redMaterialPtr->SetDiffuseColor(glm::vec3(1.0f, 0.0f, 0.0f));
		
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, cubeMesh, redMaterialPtr);
	}
	void UserUpdate(Mona::World& world, float timeStep) noexcept override {

		m_transform->Rotate(glm::vec3(0.0f,0.0f,1.0f), m_rotationSpeed * timeStep);

		auto& input = world.GetInput();

		float deltaMovement = m_speed * timeStep;

		if (input.IsKeyPressed(MONA_KEY_W)) {
			glm::vec3 translation = m_transform->GetLocalTranslation();
			m_transform->SetTranslation(translation + deltaMovement * glm::vec3(0.f, 1.f, 0.f));
		}
		else if (input.IsKeyPressed(MONA_KEY_A)) {
			glm::vec3 translation = m_transform->GetLocalTranslation();
			m_transform->SetTranslation(translation + deltaMovement * glm::vec3(-1.f, 0.f, 0.f));
		}
		else if (input.IsKeyPressed(MONA_KEY_S)) {
			glm::vec3 translation = m_transform->GetLocalTranslation();
			m_transform->SetTranslation(translation + deltaMovement * glm::vec3(0.f, -1.f, 0.f));
		}
		else if (input.IsKeyPressed(MONA_KEY_D)) {
			glm::vec3 translation = m_transform->GetLocalTranslation();
			m_transform->SetTranslation(translation + deltaMovement * glm::vec3(1.f, 0.f, 0.f));
		}
	}

private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_staticMesh;
	float m_speed;
	float m_rotationSpeed;
	
};

class Sandbox : public Mona::Application
{
public:
	Sandbox()
	{
		m_fullscreen = false;
		m_showCursor = true;
	}
	~Sandbox() = default;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		MONA_LOG_INFO("Starting User App: Sandbox");
		world.SetAmbientLight(glm::vec3(0.0f));

		auto& config = Mona::Config::GetInstance();
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& eventManager = world.GetEventManager();

		//eventManager.Subscribe(m_windowResizeSubcription, this, &Sandbox::OnWindowResize);
		//eventManager.Subscribe(m_debugGUISubcription, this, &Sandbox::OnDebugGUIEvent);
		m_rotatingBox = world.CreateGameObject<Box>(10.f, 1.0f);

		// right handed axis for coordinate system reference
		auto rightHandedAxis = world.CreateGameObject<Mona::GameObject>();
		world.AddComponent<Mona::TransformComponent>(rightHandedAxis);
		auto axisMesh = meshManager.LoadMesh(config.getPathOfEngineAsset("Models/axis-right-handed.gltf"), true);
		auto axisMaterial = world.CreateMaterial(Mona::MaterialType::DiffuseFlat);
		auto axisMaterialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(axisMaterial);
		world.AddComponent<Mona::StaticMeshComponent>(rightHandedAxis, axisMesh, axisMaterialPtr);
		
		m_camera = world.CreateGameObject<Mona::GameObject>();
		auto cameraTransform = world.AddComponent<Mona::TransformComponent>(m_camera);
		cameraTransform->SetTranslation(glm::vec3(0.0f, 2.0f, 20.0f));
		auto loc = cameraTransform->GetLocalTranslation();
		MONA_LOG_INFO("loc=({}, {}, {})", loc.x, loc.y, loc.z);
		cameraTransform->Rotate(glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-90.0f));
		auto cameraComponent = world.AddComponent<Mona::CameraComponent>(m_camera);
		world.SetMainCamera(cameraComponent);

		auto lightObject = world.CreateGameObject<Mona::GameObject>();
		auto lightTransform = world.AddComponent<Mona::TransformComponent>(lightObject);
		lightTransform->Rotate(glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-135.0f));
		auto directionalLight = world.AddComponent<Mona::DirectionalLightComponent>(lightObject, 10.f * glm::vec3(1.0f));
		directionalLight->SetLightColor(glm::vec3(1.0f, 1.0f, 1.0f));
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: Sandbox");
		//auto& eventManager = world.GetEventManager();
		//eventManager.Unsubscribe(m_debugGUISubcription);
		//eventManager.Unsubscribe(m_windowResizeSubcription);
	}
#if 0
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
#endif

	void OnWindowResize(const Mona::WindowResizeEvent& event)
	{
		MONA_LOG_INFO("A WindowResizeEvent has ocurred! {0} {1}", event.width, event.height);
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		if (input.IsKeyPressed(MONA_KEY_ESCAPE)) {
			exit(EXIT_SUCCESS);
		}
		else if (input.IsKeyPressed(MONA_KEY_F))
		{
			m_fullscreen = not m_fullscreen;
			window.SetFullScreen(m_fullscreen);
		}
		else if (input.IsKeyPressed(MONA_KEY_G))
		{
			window.SetWindowDimensions(glm::ivec2(1000, 1000));
		}
		else if (input.IsKeyPressed(MONA_KEY_H)) {
			m_showCursor = not m_showCursor;

			if (m_showCursor)
				input.SetCursorType(Mona::Input::CursorType::Normal);
			else
				input.SetCursorType(Mona::Input::CursorType::Disabled);
		}
		/*
		else if (input.IsKeyPressed(MONA_KEY_1)) {
			m_camera->SetActive(false);
			input.SetCursorType(Mona::Input::CursorType::Normal);
		}
		else if (input.IsKeyPressed(MONA_KEY_2)) {
			m_camera->SetActive(true);
			input.SetCursorType(Mona::Input::CursorType::Disabled);
		}*/
	}
private:
	//Mona::SubscriptionHandle m_windowResizeSubcription;
	//Mona::SubscriptionHandle m_debugGUISubcription;
	Mona::GameObjectHandle<Box> m_rotatingBox;
	Mona::GameObjectHandle<Mona::GameObject> m_camera;
	float somefloat = 0.0f;
	int m_currentMaterialIndex;
	bool m_fullscreen;
	bool m_showCursor;
};
int main()
{	
	Sandbox app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
}

