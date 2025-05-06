#include <MonaEngine.hpp>
#include <Utilities/BasicCameraControllers.hpp>
#include <Utilities/EngineGameObjects.hpp>
#include <Rendering/PBRTexturedMaterial.hpp>
#include <Rendering/UnlitTexturedMaterial.hpp>
#include <Rendering/DiffuseTexturedMaterial.hpp>
#include <Rendering/DiffuseFlatMaterial.hpp>
#include <Rendering/UnlitFlatMaterial.hpp>
#include <imgui.h>
#include <glm/gtx/string_cast.hpp>

class Box : public Mona::GameObject {
public:
	Box(float speed, float rotationSpeed)
	{
		m_speed = speed;
		m_rotationSpeed = rotationSpeed;
	}

	void UserStartUp(Mona::World& world) noexcept override
	{
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->SetTranslation(glm::vec3(0, 0, 0));
		m_transform->SetScale(glm::vec3(4, 4, 4));

		auto& config = Mona::Config::GetInstance();
		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& textureManager = Mona::TextureManager::GetInstance();

		auto cubeMesh = meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube);

		auto material = world.CreateMaterial(Mona::MaterialType::DiffuseFlat);
		auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(material);
		materialPtr->SetDiffuseColor(glm::vec3(1,0,0));
		
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, cubeMesh, materialPtr);
	}

	void UserUpdate(Mona::World& world, float timeStep) noexcept override
	{
		m_transform->Rotate(glm::vec3(0.0f,0.0f,1.0f), m_rotationSpeed * timeStep);
	}

private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_staticMesh;
	float m_speed;
	float m_rotationSpeed;
	
};

class Boo : public Mona::GameObject {
	public:
		Boo()
		{
		}

		void UserStartUp(Mona::World& world) noexcept override
		{
			m_transform = world.AddComponent<Mona::TransformComponent>(*this);
			m_transform->SetTranslation(glm::vec3(0, 0, 0));
			//m_transform->SetRotation(glm::angleAxis(glm::radians(90.f), glm::vec3(1, 0, 0)));
			m_transform->SetScale(glm::vec3(4, 4, 4));
	
			auto& config = Mona::Config::GetInstance();
			auto& meshManager = Mona::MeshManager::GetInstance();
			auto& textureManager = Mona::TextureManager::GetInstance();

			auto booTexture = textureManager.LoadTexture(config.getPathOfEngineAsset("Textures/boo.png"));
			auto quadMesh = meshManager.LoadMesh(config.getPathOfEngineAsset("Models/textured_quad.obj"));
	
			auto material = world.CreateMaterial(Mona::MaterialType::UnlitTextured);
			auto materialPtr = std::static_pointer_cast<Mona::UnlitTexturedMaterial>(material);
			booTexture->SetMagnificationFilter(Mona::TextureMagnificationFilter::Nearest);
			materialPtr->SetUnlitColorTexture(booTexture);
			
			m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, quadMesh, materialPtr);
		}

		void UserUpdate(Mona::World& world, float timeStep) noexcept override
		{
			//m_transform->Rotate(glm::vec3(0.0f,0.0f,1.0f), m_rotationSpeed * timeStep);
		}
	
	private:
		Mona::TransformHandle m_transform;
		Mona::StaticMeshHandle m_staticMesh;		
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

		world.SetBackgroundColor(0.5f, 0.5f, 0.5f);

		//eventManager.Subscribe(m_windowResizeSubcription, this, &Sandbox::OnWindowResize);
		//eventManager.Subscribe(m_debugGUISubcription, this, &Sandbox::OnDebugGUIEvent);
		m_rotatingBox = world.CreateGameObject<Box>(10.f, 1.0f);

		m_boo = world.CreateGameObject<Boo>();
		Mona::TransformHandle booTransformHandle = world.GetComponentHandle<Mona::TransformComponent>(m_boo);
		booTransformHandle->SetTranslation(glm::vec3(0,0,10));
		booTransformHandle->SetScale(glm::vec3(10));

		m_axis = world.CreateGameObject<Mona::Axis>();
		
		/*
		m_camera = world.CreateGameObject<Mona::GameObject>();
		auto cameraTransform = world.AddComponent<Mona::TransformComponent>(m_camera);
		cameraTransform->SetTranslation(glm::vec3(0.0f, 0.0f, 20.0f));
		auto loc = cameraTransform->GetLocalTranslation();
		MONA_LOG_INFO("loc=({}, {}, {})", loc.x, loc.y, loc.z);
		cameraTransform->Rotate(glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-80.0f));
		auto cameraComponent = world.AddComponent<Mona::CameraComponent>(m_camera);
		world.SetMainCamera(cameraComponent);
		*/

		m_camera = world.CreateGameObject<Mona::FlyingCamera>();
		m_camera->SetActive(false); // fixed camera to start
		Mona::TransformHandle cameraTransformHandle = world.GetComponentHandle<Mona::TransformComponent>(m_camera);
		cameraTransformHandle->SetTranslation(glm::vec3(42.864922, 39.083099, 20.000000));

		// this transform is looking at the center. You can get specific values printed by pressing 1 while flying around.
		// Camera Position : vec3(42.864922, 39.083099, 20.000000) - Rotation : quat(0.410619, { -0.050418, -0.110951, 0.903626 })
		cameraTransformHandle->SetRotation(glm::fquat(0.410619, -0.050418, -0.110951, 0.903626));
		
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));

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
		else if (input.IsKeyPressed(MONA_KEY_1)) {
			Mona::TransformHandle transformHandle = world.GetComponentHandle<Mona::TransformComponent>(m_camera);
			glm::vec3 cameraPosition = transformHandle->GetLocalTranslation();
			glm::fquat cameraRotation = transformHandle->GetLocalRotation();
			std::string cameraPositionStr = glm::to_string(cameraPosition);
			std::string cameraRotationStr = glm::to_string(cameraRotation);

			glm::vec3 cameraRotationEulerAngles = glm::eulerAngles(cameraRotation);
			std::string cameraRotationEulerAnglesStr = glm::to_string(cameraRotationEulerAngles);

			MONA_LOG_INFO("Camera Position: {} - Rotation: {} - EulerAngles: {}", cameraPositionStr.c_str(), cameraRotationStr.c_str(), cameraRotationEulerAnglesStr.c_str());
		}
		else if (input.IsKeyPressed(MONA_KEY_2)) {
			bool currentActiveState = m_camera->GetActive();
			m_camera->SetActive(not currentActiveState);
			MONA_LOG_INFO("Camera Active: {}", m_camera->GetActive());
		}
	}
private:
	//Mona::SubscriptionHandle m_windowResizeSubcription;
	//Mona::SubscriptionHandle m_debugGUISubcription;
	Mona::GameObjectHandle<Box> m_rotatingBox;
	Mona::GameObjectHandle<Boo> m_boo;
	Mona::GameObjectHandle<Mona::Axis> m_axis;
	Mona::GameObjectHandle<Mona::FlyingCamera> m_camera;
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

