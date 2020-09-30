#include "MonaEngine.hpp"
class BasicCamera : public Mona::GameObject {
public:
	BasicCamera() = default;
	~BasicCamera() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto transform = world.AddComponent<Mona::TransformComponent>(*this, glm::vec3(0.0f,-15.0f, 15.0f));
		transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.5f);
		world.AddComponent<Mona::CameraComponent>(*this);
		world.SetMainCamera(*this);
	}
};
class Ball : public::Mona::GameObject {
public:
	Ball() = default;
	~Ball() = default;

};
class Paddle : public Mona::GameObject {
public:
	Paddle(float velocity) : m_paddleVelocity(velocity) {}
	~Paddle() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		MONA_LOG_INFO("sdfsdfds");
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		glm::vec3 paddleScale(2.0f, 0.5f, 0.5f);
		m_transform->Scale(paddleScale);
		world.AddComponent<Mona::StaticMeshComponent>(*this, Mona::ModelManager::PrimitiveType::Cube, glm::vec3(0.9f, 0.5f, 0.3f));
		Mona::BoxShapeInformation boxInfo(paddleScale);
		Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(*this, boxInfo, Mona::RigidBodyType::KinematicBody);
		rb->SetFriction(0.0f);
		rb->SetRestitution(1.0f);
		
		auto ball = world.CreateGameObject<Ball>();
		float ballRadius = 0.5f;
		m_ballTransform = world.AddComponent<Mona::TransformComponent>(ball);
		m_ballTransform->SetRotation(m_transform->GetLocalRotation());
		m_ballTransform->SetTranslation(m_transform->GetLocalTranslation() + glm::vec3(0.0f, 2.0f, 0.0f));
		m_ballTransform->SetScale(glm::vec3(ballRadius));
		
		world.AddComponent<Mona::StaticMeshComponent>(ball, Mona::ModelManager::PrimitiveType::Sphere, glm::vec3(0.3f, 0.3f, 0.85f));
		
		Mona::SphereShapeInformation sphereInfo(ballRadius);
		m_ballRigidBody = world.AddComponent<Mona::RigidBodyComponent>(ball, sphereInfo, Mona::RigidBodyType::DynamicBody);
		m_ballRigidBody->SetRestitution(1.0f);
		m_ballRigidBody->SetFriction(0.0f);
		
	}

	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		auto& input = world.GetInput();
		if (input.IsKeyPressed(MONA_KEY_A))
		{
			m_transform->Translate(glm::vec3(-m_paddleVelocity * timeStep, 0.0f, 0.0f));
		}
		else if (input.IsKeyPressed(MONA_KEY_D))
		{
			m_transform->Translate(glm::vec3(m_paddleVelocity * timeStep, 0.0f, 0.0f));
		}

		if (input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1)) {
			m_ballRigidBody->SetLinearVelocity(glm::vec3(0.0f,15.0f,0.0f));
		}
	}
private:
	Mona::TransformHandle m_transform;
	Mona::TransformHandle m_ballTransform;
	Mona::RigidBodyHandle m_ballRigidBody;
	float m_paddleVelocity;
};

class Block : public Mona::GameObject {
public:
	Block() = default;
	~Block() = default;
};

class Wall : public Mona::GameObject {
public:
	Wall() = default;
	~Wall() = default;
};
void InitializeWall(Mona::World &world, Mona::GameObjectHandle<Wall>& wall, const glm::vec3& position, const glm::vec3& scale) {
	world.AddComponent<Mona::TransformComponent>(wall, position, glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), scale);
	world.AddComponent<Mona::StaticMeshComponent>(wall, Mona::ModelManager::PrimitiveType::Cube, glm::vec3(0.15f));
	Mona::BoxShapeInformation wallShape(scale);
	Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(wall, wallShape, Mona::RigidBodyType::StaticBody);
	rb->SetRestitution(1.0f);
	rb->SetFriction(0.0f);
}
class Breakout : public Mona::Application {
public:
	Breakout() = default;
	~Breakout() = default;
	virtual void UserStartUp(Mona::World & world) noexcept override {
		world.SetGravity(glm::vec3(0.0f,0.0f,0.0f));
		world.CreateGameObject<BasicCamera>();
		world.CreateGameObject<Paddle>(20.0f);
		glm::vec3 blockScale(1.0f, 0.5f, 0.5f);
		Mona::BoxShapeInformation boxInfo(blockScale);
		for (int i = -2; i < 3; i++) {
			float x = 4.0f * i;
			for (int j = -2; j < 3; j++)
			{
				float y = 2.0f * j;
				auto block = world.CreateGameObject<Block>();
				auto transform = world.AddComponent<Mona::TransformComponent>(block, glm::vec3( x, 15.0f + y, 0.0f));
				transform->Scale(blockScale);
				world.AddComponent<Mona::StaticMeshComponent>(block, Mona::ModelManager::PrimitiveType::Cube);
				Mona::RigidBodyHandle rb =world.AddComponent<Mona::RigidBodyComponent>(block, boxInfo, Mona::RigidBodyType::StaticBody);
				rb->SetRestitution(1.0f);
				rb->SetFriction(0.0f);
			}
		}
		auto upperWall = world.CreateGameObject<Wall>();
		InitializeWall(world, upperWall, glm::vec3(0.0f, 26.0f, 0.0f), glm::vec3(18.0f, 1.0f, 1.0f));

		glm::vec3 sideWallScale(1.0f, 27.0f, 1.0f);
		float sideWallOffset = 19.0f;
		auto leftWall = world.CreateGameObject<Wall>();
		InitializeWall(world, leftWall, glm::vec3(-sideWallOffset, 0.0f, 0.0f), sideWallScale);
		auto rightWall = world.CreateGameObject<Wall>();
		InitializeWall(world, rightWall, glm::vec3(sideWallOffset, 0.0f, 0.0f), sideWallScale);

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