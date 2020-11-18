#include "MonaEngine.hpp"
#include "Rendering/FlatColorMaterial.hpp"
class BasicCamera : public Mona::GameObject {
public:
	BasicCamera() = default;
	~BasicCamera() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto transform = world.AddComponent<Mona::TransformComponent>(*this, glm::vec3(0.0f,-15.0f, 15.0f));
		transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.5f);
		world.SetMainCamera(world.AddComponent<Mona::CameraComponent>(*this));
		world.SetAudioListenerTransform(transform);
		auto audioClipPtr = world.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/music.wav"));
		auto audioSource = world.AddComponent<Mona::AudioSourceComponent>(*this, audioClipPtr);
		audioSource->SetIsLooping(true);
		audioSource->SetVolume(0.3f);
		audioSource->Play();
	}
};

class Paddle : public Mona::GameObject {
public:
	Paddle(float velocity) : m_paddleVelocity(velocity) {}
	~Paddle() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		glm::vec3 paddleScale(2.0f, 0.5f, 0.5f);
		m_transform->Scale(paddleScale);
		auto paddleMaterial = std::static_pointer_cast<Mona::FlatColorMaterial>(world.CreateMaterial(Mona::MaterialType::FlatColor));
		paddleMaterial->SetColor(glm::vec3(0.3f, 0.3f, 0.75f));
		world.AddComponent<Mona::StaticMeshComponent>(*this, world.LoadMesh(Mona::MeshManager::PrimitiveType::Cube), paddleMaterial);
		Mona::BoxShapeInformation boxInfo(paddleScale);
		Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(*this, boxInfo, Mona::RigidBodyType::KinematicBody);
		rb->SetFriction(0.0f);
		rb->SetRestitution(1.0f);
		
		
		m_ballBounceSound = world.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/ballBounce.wav"));
		auto ball = world.CreateGameObject<Mona::GameObject>();
		float ballRadius = 0.5f;
		m_ballTransform = world.AddComponent<Mona::TransformComponent>(ball);
		m_ballTransform->SetRotation(m_transform->GetLocalRotation());
		m_ballTransform->SetTranslation(m_transform->GetLocalTranslation() + glm::vec3(0.0f, 2.0f, 0.0f));
		m_ballTransform->SetScale(glm::vec3(ballRadius));
		auto ballMaterial = std::static_pointer_cast<Mona::FlatColorMaterial>(world.CreateMaterial(Mona::MaterialType::FlatColor));
		ballMaterial->SetColor(glm::vec3(0.75f, 0.3f, 0.3f));
		world.AddComponent<Mona::StaticMeshComponent>(ball, world.LoadMesh(Mona::MeshManager::PrimitiveType::Sphere), ballMaterial);
		
		Mona::SphereShapeInformation sphereInfo(ballRadius);
		m_ballRigidBody = world.AddComponent<Mona::RigidBodyComponent>(ball, sphereInfo, Mona::RigidBodyType::DynamicBody);
		m_ballRigidBody->SetRestitution(1.0f);
		m_ballRigidBody->SetFriction(0.0f);
		auto callback = [ballTransform = m_ballTransform, ballSound = m_ballBounceSound](Mona::World& world, Mona::RigidBodyHandle& otherRigidBody, bool isSwaped, Mona::CollisionInformation& colInfo) mutable {
			world.PlayAudioClip3D(ballSound, ballTransform->GetLocalTranslation(),0.3f);
		};
		m_ballRigidBody->SetStartCollisionCallback(callback);
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
	std::shared_ptr<Mona::AudioClip> m_ballBounceSound;
	float m_paddleVelocity;
};


void InitializeWall(Mona::World &world,
	Mona::GameObjectHandle<Mona::GameObject>& wall,
	const glm::vec3& position,
	const glm::vec3& scale,
	std::shared_ptr<Mona::Material> wallMaterial) {
	world.AddComponent<Mona::TransformComponent>(wall, position, glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), scale);
	world.AddComponent<Mona::StaticMeshComponent>(wall, world.LoadMesh(Mona::MeshManager::PrimitiveType::Cube), wallMaterial);
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
		m_blockBreakingSound = world.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/boxBreaking.wav"));

		Mona::BoxShapeInformation boxInfo(blockScale);
		auto blockMaterial = world.CreateMaterial(Mona::MaterialType::FlatColor);
		for (int i = -2; i < 3; i++) {
			float x = 4.0f * i;
			for (int j = -2; j < 3; j++)
			{
				float y = 2.0f * j;
				auto block = world.CreateGameObject<Mona::GameObject>();
				auto transform = world.AddComponent<Mona::TransformComponent>(block, glm::vec3( x, 15.0f + y, 0.0f));
				transform->Scale(blockScale);
				world.AddComponent<Mona::StaticMeshComponent>(block, world.LoadMesh(Mona::MeshManager::PrimitiveType::Cube), blockMaterial);
				Mona::RigidBodyHandle rb =world.AddComponent<Mona::RigidBodyComponent>(block, boxInfo, Mona::RigidBodyType::StaticBody, 1.0f);
				rb->SetRestitution(1.0f);
				rb->SetFriction(0.0f);
				auto callback = [block, blockSound = m_blockBreakingSound](Mona::World& world, Mona::RigidBodyHandle& otherRigidBody, bool isSwaped, Mona::CollisionInformation& colInfo) mutable {
					world.PlayAudioClip2D(blockSound, 1.0f, 1.0f);
					world.DestroyGameObject(block);
				};

				rb->SetStartCollisionCallback(callback);
				
			}
		}
		auto wallMaterial = std::static_pointer_cast<Mona::FlatColorMaterial>(world.CreateMaterial(Mona::MaterialType::FlatColor));
		wallMaterial->SetColor(glm::vec3(0.15f, 0.15f, 0.15f));
		auto upperWall = world.CreateGameObject<Mona::GameObject>();
		InitializeWall(world, upperWall, glm::vec3(0.0f, 26.0f, 0.0f), glm::vec3(18.0f, 1.0f, 1.0f), wallMaterial);

		glm::vec3 sideWallScale(1.0f, 27.0f, 1.0f);
		float sideWallOffset = 19.0f;
		auto leftWall = world.CreateGameObject<Mona::GameObject>();
		InitializeWall(world, leftWall, glm::vec3(-sideWallOffset, 0.0f, 0.0f), sideWallScale, wallMaterial);
		auto rightWall = world.CreateGameObject<Mona::GameObject>();
		InitializeWall(world, rightWall, glm::vec3(sideWallOffset, 0.0f, 0.0f), sideWallScale, wallMaterial);

	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World & world, float timeStep) noexcept override {
	}
	std::shared_ptr<Mona::AudioClip> m_blockBreakingSound;
private:
	
	bool onlyOnce = true;
};
int main()
{
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::unique_ptr<Mona::Application>(new Breakout()));
	engine.StartMainLoop();
	engine.ShutDown();
}