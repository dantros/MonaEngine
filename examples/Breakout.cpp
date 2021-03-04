#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"

void CreateBasicCameraWithMusicAndLight(Mona::World& world)
{
	auto camera = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(camera, glm::vec3(0.0f, -15.0f, 15.0f));
	transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.5f);
	world.SetMainCamera(world.AddComponent<Mona::CameraComponent>(camera));
	world.SetAudioListenerTransform(transform);
	auto& audioClipManager = Mona::AudioClipManager::GetInstance();
	auto audioClipPtr = audioClipManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/music.wav"));
	auto audioSource = world.AddComponent<Mona::AudioSourceComponent>(camera, audioClipPtr);
	audioSource->SetIsLooping(true);
	audioSource->SetVolume(0.3f);
	audioSource->Play();

	world.AddComponent<Mona::DirectionalLightComponent>(camera, glm::vec3(1.0f));

}

void CreateWall(Mona::World& world,
	const glm::vec3& position,
	const glm::vec3& scale,
	std::shared_ptr<Mona::Material> wallMaterial) {
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto wall = world.CreateGameObject<Mona::GameObject>();
	world.AddComponent<Mona::TransformComponent>(wall, position, glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), scale);
	world.AddComponent<Mona::StaticMeshComponent>(wall, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), wallMaterial);
	Mona::BoxShapeInformation wallShape(scale);
	Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(wall, wallShape, Mona::RigidBodyType::StaticBody);
	rb->SetRestitution(1.0f);
	rb->SetFriction(0.0f);
}


class Paddle : public Mona::GameObject {
public:
	Paddle(float velocity) : m_paddleVelocity(velocity) {}
	~Paddle() = default;
	virtual void UserStartUp(Mona::World& world) noexcept {
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		glm::vec3 paddleScale(2.0f, 0.5f, 0.5f);
		m_transform->Scale(paddleScale);
		auto paddleMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		paddleMaterial->SetDiffuseColor(glm::vec3(0.3f, 0.3f, 0.75f));
		auto& meshManager = Mona::MeshManager::GetInstance();
		world.AddComponent<Mona::StaticMeshComponent>(*this, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), paddleMaterial);
		Mona::BoxShapeInformation boxInfo(paddleScale);
		Mona::RigidBodyHandle rb = world.AddComponent<Mona::RigidBodyComponent>(*this, boxInfo, Mona::RigidBodyType::KinematicBody);
		rb->SetFriction(0.0f);
		rb->SetRestitution(1.0f);
		
		auto& audioClipManager = Mona::AudioClipManager::GetInstance();
		m_ballBounceSound = audioClipManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/ballBounce.wav"));

		auto ball = world.CreateGameObject<Mona::GameObject>();
		float ballRadius = 0.5f;
		m_ballTransform = world.AddComponent<Mona::TransformComponent>(ball);
		m_ballTransform->SetRotation(m_transform->GetLocalRotation());
		m_ballTransform->SetTranslation(m_transform->GetLocalTranslation() + glm::vec3(0.0f, 2.0f, 0.0f));
		m_ballTransform->SetScale(glm::vec3(ballRadius));
		auto ballMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		ballMaterial->SetDiffuseColor(glm::vec3(0.75f, 0.3f, 0.3f));
		world.AddComponent<Mona::StaticMeshComponent>(ball, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Sphere), ballMaterial);
		
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



class Breakout : public Mona::Application {
public:
	Breakout() = default;
	~Breakout() = default;
	virtual void UserStartUp(Mona::World & world) noexcept override {
		world.SetGravity(glm::vec3(0.0f,0.0f,0.0f));
		world.SetAmbientLight(glm::vec3(0.3f));
		CreateBasicCameraWithMusicAndLight(world);
		world.CreateGameObject<Paddle>(20.0f);
		

		//Crear el los bloques destructibles del nivel
		glm::vec3 blockScale(1.0f, 0.5f, 0.5f);
		auto& audioClipManager = Mona::AudioClipManager::GetInstance();
		m_blockBreakingSound = audioClipManager.LoadAudioClip(Mona::SourcePath("Assets/AudioFiles/boxBreaking.wav"));
		auto& meshManager = Mona::MeshManager::GetInstance();
		Mona::BoxShapeInformation boxInfo(blockScale);
		auto blockMaterial = world.CreateMaterial(Mona::MaterialType::DiffuseFlat);
		for (int i = -2; i < 3; i++) {
			float x = 4.0f * i;
			for (int j = -2; j < 3; j++)
			{
				float y = 2.0f * j;
				auto block = world.CreateGameObject<Mona::GameObject>();
				auto transform = world.AddComponent<Mona::TransformComponent>(block, glm::vec3( x, 15.0f + y, 0.0f));
				transform->Scale(blockScale);
				world.AddComponent<Mona::StaticMeshComponent>(block, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), blockMaterial);
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

		//Crear las paredes
		auto wallMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		wallMaterial->SetDiffuseColor(glm::vec3(0.15f, 0.15f, 0.15f));


		CreateWall(world, glm::vec3(0.0f, 26.0f, 0.0f), glm::vec3(18.0f, 1.0f, 1.0f), wallMaterial);

		glm::vec3 sideWallScale(1.0f, 27.0f, 1.0f);
		float sideWallOffset = 19.0f;
		CreateWall(world, glm::vec3(-sideWallOffset, 0.0f, 0.0f), sideWallScale, wallMaterial);
		CreateWall(world, glm::vec3(sideWallOffset, 0.0f, 0.0f), sideWallScale, wallMaterial);

	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World & world, float timeStep) noexcept override {
	}
	std::shared_ptr<Mona::AudioClip> m_blockBreakingSound;
};
int main()
{
	Breakout breakout;
	Mona::Engine engine(breakout);
	engine.StartMainLoop();
}