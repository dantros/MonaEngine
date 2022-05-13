#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include <numbers>



float gaussian(float x, float y, float s, float sigma, glm::vec2 mu) {
	return (s / (sigma * std::sqrt(2 * std::numbers::pi))) * std::exp((-1 / (2 * std::pow(sigma, 2))) * (std::pow((x - mu[0]), 2) + std::pow((y - mu[1]), 2)));
}

void CreateTerrain(Mona::World& world) {
	auto terrain = world.CreateGameObject<Mona::GameObject>();
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
	materialPtr->SetDiffuseColor(glm::vec3(0.3, 0.5f, 0.7f));
	float planeScale = 1.0f;
	auto transform = world.AddComponent<Mona::TransformComponent>(terrain);
	transform->SetScale(glm::vec3(planeScale));

	auto heightFun = [](float x, float y) {
		return (gaussian(x, y, 30, 5, { -10, 0 }) + gaussian(x, y, 50, 3, { 10, 0 }));
	};
	auto terrainMesh = meshManager.GenerateTerrain({ -10,-10 }, { 10,10 }, 20, 20, heightFun);

	world.AddComponent<Mona::StaticMeshComponent>(terrain, terrainMesh, materialPtr);
	Mona::BoxShapeInformation boxInfo(glm::vec3(planeScale, planeScale, planeScale));
	Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(terrain, boxInfo, Mona::RigidBodyType::StaticBody, 1.0f, false, glm::vec3(0.0f, 0.0f, -planeScale));
}

void AddDirectionalLight(Mona::World& world, const glm::vec3& axis, float angle, const glm::vec3& center, float lightIntensity)
{
	auto light = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(light);
	transform->SetTranslation(center);
	transform->Rotate(axis, angle);
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}
class TerrainRender : public Mona::Application
{
public:
	TerrainRender() = default;
	~TerrainRender() = default;
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera> m_camera;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera>();
		world.SetAmbientLight(glm::vec3(0.2f));
		world.AddComponent<Mona::SpotLightComponent>(m_camera, glm::vec3(100.0f), 5.0f, glm::radians(25.0f), glm::radians(37.0f));
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		CreateTerrain(world);
		//AddDirectionalLight(world, glm::vec3(0.0f, 0.0f, 10.0f), 2.0f, glm::radians(-45.0f));
		AddDirectionalLight(world, glm::vec3(0.0f, 1.0f, 0.0f), glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 100.0f), 15.0f);

		
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
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
};
int main()
{	
	TerrainRender app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
	
}