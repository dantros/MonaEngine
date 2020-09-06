#include "MonaEngine.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include <imgui.h>
class Box : public Mona::GameObject {
public:
	Box(float speed, float rspeed) {
		m_speed = speed;
		m_rotationSpeed = rspeed;
	}
	void UserStartUp(Mona::World& world) noexcept override {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this);
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

class Sandbox : public Mona::Application
{
public:
	Sandbox() = default;
	~Sandbox() = default;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		MONA_LOG_INFO("Starting User App: Sandbox");
		auto& eventManager = world.GetEventManager();
		m_windowResizeSubcription = eventManager.Subscribe(this, &Sandbox::OnWindowResize);
		m_debugGUISubcription = eventManager.Subscribe(this, &Sandbox::OnDebugGUIEvent);
		world.CreateGameObject<Box>(0.4f, 0.0f);
		m_rotatingBox = world.CreateGameObject<Box>(0.0f, 0.0f);
		world.SetMainCamera(world.CreateGameObject<Mona::BasicPerspectiveCamera>());
		world.GetInput().SetCursorType(Mona::Input::CursorType::Disabled);
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