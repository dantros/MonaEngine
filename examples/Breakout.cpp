#include "MonaEngine.hpp"
class BasicCamera : public Mona::GameObject {
public:
	BasicCamera() = default;
	~BasicCamera() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto& transform = world.AddComponent<Mona::TransformComponent>(*this, glm::vec3(0.0f,-15.0f, 15.0f));
		transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.5f);
		world.AddComponent<Mona::CameraComponent>(*this);
		world.SetMainCamera(*this);
	}
};

class Paddle : public Mona::GameObject {
public:
	Paddle(float velocity) : m_paddleVelocity(velocity) {}
	~Paddle() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->Scale(glm::vec3(1.5f, 0.2f, 0.2f));
		world.AddComponent<Mona::StaticMeshComponent>(*this, Mona::ModelManager::PrimitiveType::Cube, glm::vec3(0.9f, 0.5f, 0.3f));
	}

	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		auto& input = world.GetInput();
		if (input.IsKeyPressed(MONA_KEY_A))
		{
			m_transform->Translate(glm::vec3(-m_paddleVelocity*timeStep, 0.0f,0.0f));
		}
		else if (input.IsKeyPressed(MONA_KEY_D))
		{
			m_transform->Translate(glm::vec3(m_paddleVelocity * timeStep, 0.0f, 0.0f));
		}
	}
private:
	Mona::TransformHandle m_transform;
	float m_paddleVelocity;
};

class Block : public Mona::GameObject {
public:
	Block() = default;
	~Block() = default;
};

class Breakout : public Mona::Application {
public:
	Breakout() = default;
	~Breakout() = default;
	virtual void UserStartUp(Mona::World & world) noexcept override {
		world.CreateGameObject<BasicCamera>();
		world.CreateGameObject<Paddle>(5.0f);
		for (int i = -2; i < 3; i++) {
			float x = 2.5f * i;
			for (int j = -2; j < 3; j++)
			{
				float y = 1.5f * j;
				auto& block = world.CreateGameObject<Block>();
				auto& transform = world.AddComponent<Mona::TransformComponent>(block, glm::vec3( x, 15.0f + y, 0.0f));
				transform->Scale(glm::vec3(1.0f, 0.2f, 0.2f));
				world.AddComponent<Mona::StaticMeshComponent>(block, Mona::ModelManager::PrimitiveType::Cube);
			}
		}

	}

	virtual void UserShutDown(Mona::World& world) noexcept override {}
	virtual void UserUpdate(Mona::World & world, float timeStep) noexcept override {}
};
int main()
{
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Breakout()));
	engine.StartMainLoop();
	engine.ShutDown();
}