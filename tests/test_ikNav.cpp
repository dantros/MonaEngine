#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include <numbers>



float gaussian(float x, float y, float s, float sigma, glm::vec2 mu) {
	return (s / (sigma * std::sqrt(2 * std::numbers::pi))) * std::exp((-1 / (2 * std::pow(sigma, 2))) * (std::pow((x - mu[0]), 2) + std::pow((y - mu[1]), 2)));
}

void AddDirectionalLight(Mona::World& world, const glm::vec3& axis, float angle, float lightIntensity)
{
	auto light = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(light);
	transform->Rotate(axis, angle);
	//transform->SetTranslation(center);
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}

void AddTerrain() {

}



class IKNav : public Mona::Application
{
public:
	IKNav() = default;
	~IKNav() = default;
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera> m_camera;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera>();
		world.SetAmbientLight(glm::vec3(0.2f));
		//world.AddComponent<Mona::SpotLightComponent>(m_camera, glm::vec3(100.0f), 5.0f, glm::radians(25.0f), glm::radians(37.0f));
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		//AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-180.0f), glm::vec3(0.0f, 0.0f, 1000.0f),5.0f);
		//AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1000.0f),5.0f);
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-45.0f), 2);
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-135.0f), 2);
		//AddDirectionalLight(world, glm::vec3(0.0f, 1.0f, 0.0f), glm::radians(-135.0f), 15.0f);

		
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
	IKNav app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
	
}