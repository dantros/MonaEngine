#include "MonaEngine.hpp"
#include <imgui.h>
class Box : public Mona::GameObject {
public:
	Box() = default;
	void UserStartUp(Mona::World& world) noexcept override {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this);
	}
	void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		m_transform->Translate(glm::vec3(0.1f)*timeStep);
	}
private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_staticMesh;
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
		world.CreateGameObject<Box>();
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: Sandbox");
		auto& eventManager = world.GetEventManager();
		eventManager.Unsubscribe(m_debugGUISubcription);
		eventManager.Unsubscribe(m_windowResizeSubcription);
	}
	
	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Testing Float Slider:");
		ImGui::SliderFloat("SomeFloat", &somefloat, 0.0f, 10.0f);
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
	float somefloat = 0.0f;
};
int main()
{	
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Sandbox()));
	engine.StartMainLoop();
	engine.ShutDown();
}