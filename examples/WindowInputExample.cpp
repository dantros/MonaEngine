#include "MonaEngine.hpp"

void OnWindowResizeFreeFunction(const Mona::WindowResizeEvent& event)
{
	MONA_LOG_INFO("A WindowResizeEvent has ocurred called from free function! {0} {1}", event.width, event.height);
}
class Sandbox : public Mona::Application
{
public:
	Sandbox() = default;
	~Sandbox() = default;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		MONA_LOG_INFO("Starting User App: Sandbox");
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(this, &Sandbox::OnWindowResize);
		eventManager.Subscribe(&OnWindowResizeFreeFunction);
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: Sandbox");
	}
	void OnWindowResize(const Mona::WindowResizeEvent& event)
	{
		MONA_LOG_INFO("A WindowResizeEvent has ocurred! {0} {1}", event.width, event.height);
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		if (m_input->IsKeyPressed(MONA_KEY_G))
		{
			m_window->SetFullScreen(true);
		}
		else if (m_input->IsKeyPressed(MONA_KEY_H))
		{
			m_window->SetFullScreen(false);
		}
		else if (m_input->IsKeyPressed(MONA_KEY_J))
		{
			m_window->SetWindowDimensions(glm::ivec2(1000, 1000));
		}
		else if (m_input->IsKeyPressed(MONA_KEY_D)) {
		}
		else if (m_input->GetMouseWheelOffset().y > 0.0)
		{
			auto Offset = m_input->GetMouseWheelOffset();
			MONA_LOG_INFO("The mouse offset is ({0},{1})", Offset.x, Offset.y);

		}
	}
private:
	Mona::SubscriptionHandle m_windowResizeSubcription;
};
int main()
{	
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Sandbox()));
	engine.StartMainLoop();
	engine.ShutDown();
}