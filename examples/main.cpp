#include "MonaEngine.hpp"

class MyBox : public Mona::GameObject
{
public:
	virtual void Start() noexcept override{
		MONA_LOG_INFO("Starting Box Object");
		counter =  0;
		m_transformComponent = Mona::AddComponent<Mona::TransformComponent>(*this);
		//auto m_cameraComponent = AddComponent<Mona::StaticMeshComponent>();
	}
	virtual void Update(float timeStep) noexcept override
	{
		if (counter > 300)
		{
			MONA_LOG_INFO("Updating MyBox Object: ID {0} , counter = {1}", GetObjectID(), counter);
			auto translation = m_transformComponent->GetLocalTranslation();
			MONA_LOG_INFO("Current Transform Position ({0},{1},{2})", translation.x, translation.y, translation.z);
			counter = 0;
		}

		m_transformComponent->Translate(glm::vec3(timeStep));
		counter++;
	}
	~MyBox() {
		MONA_LOG_INFO("Callling Box Deconstructor");
	}
private:
	int counter;
	Mona::ComponentHandle<Mona::TransformComponent> m_transformComponent;

};

class Sandbox : public Mona::Application
{
public:
	Sandbox() = default;
	~Sandbox() = default;
	virtual void UserStartUp() noexcept override{
		MONA_LOG_INFO("Starting User App: Sandbox");
		auto& world = Mona::World::GetInstance();
		m_boxObject = world.CreateGameObject<MyBox>();
		m_boxObject2 = world.CreateGameObject<MyBox>();
	}

	virtual void UserShutDown() noexcept override {
		MONA_LOG_INFO("ShuttingDown User App: Sandbox");
	}

	virtual void UserUpdate(float timeStep) noexcept override {
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
			Mona::World::GetInstance().DestroyGameObject(m_boxObject);
		}
		else if (m_input->GetMouseWheelOffset().y > 0.0)
		{
			auto Offset = m_input->GetMouseWheelOffset();
			MONA_LOG_INFO("The mouse offset is ({0},{1})", Offset.x, Offset.y);

		}
	}
private:
	std::weak_ptr<Mona::GameObject> m_boxObject;
	std::weak_ptr<Mona::GameObject> m_boxObject2;
};
int main()
{	
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Sandbox()));
	engine.StartMainLoop();
	engine.ShutDown();
}