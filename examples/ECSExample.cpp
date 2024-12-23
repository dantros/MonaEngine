#include "MonaEngine.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "ECS/ECS.hpp"
#include "ECS/Components.hpp"
#include "ECS/Events.hpp"
#include "ECS/Systems.hpp"
#include <random>

void CreateCamera(Mona::World &world)
{
	auto camera = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(camera, glm::vec3(0.0f, -50.0f, 20.0f));
	transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.0f);
	world.SetMainCamera(world.AddComponent<Mona::CameraComponent>(camera));
}


class StaticBox : public Mona::GameObject
{
public:
	StaticBox(glm::vec3 translation, glm::vec3 scale) : m_translation(translation), m_scale(scale) {}
	~StaticBox() = default;
	virtual void UserStartUp(Mona::World &world) noexcept
	{
		auto box = world.CreateGameObject<Mona::GameObject>();
		
		m_boxTransform = world.AddComponent<Mona::TransformComponent>(box);
		m_boxTransform->SetTranslation(m_translation);
		m_boxTransform->SetScale(m_scale);

		auto ecs = world.GetECSHandler();
		auto e = ecs->CreateEntity();
		boxEntity = e;
		ecs->AddComponent<MonaECS::TransformComponent>(e, &m_boxTransform);
		ecs->AddComponent<MonaECS::ColliderComponent>(e, m_scale, false);
		ecs->AddComponent<MonaECS::BodyComponent>(e, glm::vec3(0.0f), glm::vec3(0.0f), 1.0f);

		auto boxMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		boxMaterial->SetDiffuseColor(glm::vec3(0.75f, 0.3f, 0.3f));
		world.AddComponent<Mona::StaticMeshComponent>(box, Mona::MeshManager::GetInstance().LoadMesh(Mona::Mesh::PrimitiveType::Cube), boxMaterial);
	}

	entt::entity GetBoxEntity()
	{
		return boxEntity;
	}

private:
	Mona::TransformHandle m_boxTransform;
	glm::vec3 m_translation;
	glm::vec3 m_scale;
	entt::entity boxEntity;
};


// Color system
enum Color
{
	RED,
	GREEN,
	BLUE
};

struct ColorComponent
{
	Mona::DiffuseFlatMaterial* material;
	Color color;
};


glm::vec3 getColor(Color color)
{
	switch (color)
	{
	case RED:
		return glm::vec3(1.0f, 0.0f, 0.0f);
	case GREEN:
		return glm::vec3(0.0f, 1.0f, 0.0f);
	case BLUE:
		return glm::vec3(0.0f, 0.0f, 1.0f);
	default:
		return glm::vec3(1.0f);
	}
}

Color getRandomColor()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> dis(0, 2);
	return static_cast<Color>(dis(gen));
}

class MovingBox : public Mona::GameObject
{
public:
	MovingBox(glm::vec3 translation, glm::vec3 velocity, float boxSize=1) : m_translation(translation), m_velocity(velocity), m_boxSize(boxSize) {}
	~MovingBox() = default;
	virtual void UserStartUp(Mona::World &world) noexcept
	{
		auto box = world.CreateGameObject<Mona::GameObject>();
		float boxSize = 0.5f;
		m_boxTransform = world.AddComponent<Mona::TransformComponent>(box);
		m_boxTransform->SetTranslation(m_translation);
		m_boxTransform->SetScale(glm::vec3(m_boxSize));

		auto ecs = world.GetECSHandler();
		auto e = ecs->CreateEntity();
		ecs->AddComponent<MonaECS::TransformComponent>(e, &m_boxTransform);
		ecs->AddComponent<MonaECS::ColliderComponent>(e, glm::vec3(m_boxSize), true);
		ecs->AddComponent<MonaECS::BodyComponent>(e, m_velocity, glm::vec3(0.0f), 1.0f);

		auto boxMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
		auto color = getRandomColor();
		ecs->AddComponent<ColorComponent>(e, boxMaterial.get(), color);
		boxMaterial->SetDiffuseColor(getColor(color));
		world.AddComponent<Mona::StaticMeshComponent>(box, Mona::MeshManager::GetInstance().LoadMesh(Mona::Mesh::PrimitiveType::Cube), boxMaterial);

		Mona::BoxShapeInformation boxInfo(glm::vec3(boxSize));
	}

	
private:
	Mona::TransformHandle m_boxTransform;
	glm::vec3 m_translation;
	glm::vec3 m_velocity;
	float m_boxSize;
};

class ECSExample : public Mona::Application
{
public:
	ECSExample() = default;
	~ECSExample() = default;
	virtual void UserStartUp(Mona::World &world) noexcept override
	{
		world.SetAmbientLight(glm::vec3(0.3f));
		CreateCamera(world);
		ecsHandler = world.GetECSHandler();


		// Walls Setting

		float wallWidth = 1.0f;

		float topWallz = 40.0f;
		float hWallWidth = 40.0f;

		float vWallz = hWallWidth/2.0f;
		float vWallx = hWallWidth - wallWidth; // To adjust the wall with the ends of the horizontal walls

		float hScale = hWallWidth;
		float vScale = vWallz - wallWidth - 0.01f; // 0.01f is to avoid collision with the horizontal walls

		const glm::vec3 hWallScale = glm::vec3(hScale, wallWidth, wallWidth);
		const glm::vec3 vWallScale = glm::vec3(wallWidth, wallWidth, vScale);

		auto wall1 = world.CreateGameObject<StaticBox>(glm::vec3(0.0f, 0.0f, 0.0f), hWallScale);
		auto wall2 = world.CreateGameObject<StaticBox>(glm::vec3(0.0f, 0.0f, topWallz), hWallScale);
		auto wall3 = world.CreateGameObject<StaticBox>(glm::vec3(-vWallx, 0.0f, vWallz), vWallScale);
		auto wall4 = world.CreateGameObject<StaticBox>(glm::vec3(vWallx, 0.0f, vWallz), vWallScale);
		walls.push_back(wall1->GetBoxEntity());
		walls.push_back(wall2->GetBoxEntity());
		walls.push_back(wall3->GetBoxEntity());
		walls.push_back(wall4->GetBoxEntity());

		/*
		 *Boxes setting
		 */

		// Positioning the boxes
		uint16_t numBoxes = 50;
		float boxSize = 0.5f;
		float boxPadding = wallWidth + boxSize;

		float xStart = -vWallx + boxPadding;
		float xEnd = vWallx - boxPadding;
		float zStart = boxPadding;
		float zEnd = topWallz - boxPadding;

		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<float> disX(xStart, xEnd);
		std::uniform_real_distribution<float> disZ(zStart, zEnd);

		// Randomize the velocity of the boxes
		std::uniform_real_distribution<float> disVel(-4.0f, 4.0f);
		auto generateNonZeroVelocity = [&disVel, &gen]() {
			float velocity;
			do {
				velocity = disVel(gen);
			} while (velocity == 0.0f);
			return velocity*3.0f;
		};

		std::vector<glm::vec3> boxPositions;

		for (uint16_t i = 0; i < numBoxes; i++)
		{
			glm::vec3 position;
		    bool positionValid;
		
			// Generate a valid position that does not overlap with existing boxes
			do {
				positionValid = true;
				position = glm::vec3(disX(gen), 0.0f, disZ(gen));

				for (const auto& existingPosition : boxPositions) {
					if (glm::distance(position, existingPosition) < boxSize * 2.0f) {
						positionValid = false;
						break;
					}
				}
			} while (!positionValid);
			world.CreateGameObject<MovingBox>(position, glm::vec3(generateNonZeroVelocity(), 0.0f, generateNonZeroVelocity()), boxSize);
		}

		// Subscribe to collision events
		ecsHandler->SubscribeEvent<MonaECS::CollisionEvent, ECSExample, &ECSExample::OnCollision>(*this);

		// Register systems
		ecsHandler->RegisterSystem<MonaECS::StatsSystem>();
		ecsHandler->RegisterSystem<MonaECS::CollisionSystem>();
		ecsHandler->RegisterSystem<MonaECS::MovementSystem>();
		ecsHandler->RegisterSystem<MonaECS::InputSystem>();

		// Start up systems
		ecsHandler->StartUpSystems();
	}

	virtual void UserShutDown(Mona::World &world) noexcept override
	{
		ecsHandler->ShutDownSystems();
	}
	virtual void UserUpdate(Mona::World &world, float timeStep) noexcept override
	{
		ecsHandler->UpdateSystems(timeStep);
	}

	void OnCollision(const MonaECS::CollisionEvent &event)
	{
		std::cout << "Collision detected" << std::endl;
		auto e1 = event.entity1;
		auto e2 = event.entity2;
		auto normal = event.normal;

		auto &transform1 = ecsHandler->GetComponent<MonaECS::TransformComponent>(e1);
		auto &transform2 = ecsHandler->GetComponent<MonaECS::TransformComponent>(e2);

		auto &body1 = ecsHandler->GetComponent<MonaECS::BodyComponent>(e1);
		auto &body2 = ecsHandler->GetComponent<MonaECS::BodyComponent>(e2);

		if (std::find(walls.begin(), walls.end(), e1) != walls.end())
		{
			body2.velocity = glm::reflect(body2.velocity, normal);
			(*transform2.tHandle)->SetTranslation((*transform2.tHandle)->GetLocalTranslation() + body2.velocity * 0.02f);
		}		
		else if (std::find(walls.begin(), walls.end(), e2) != walls.end())
		{
			body1.velocity = glm::reflect(body1.velocity, normal);
			(*transform1.tHandle)->SetTranslation((*transform1.tHandle)->GetLocalTranslation() + body1.velocity * 0.02f);
		}
		else
		{
			body1.velocity = glm::reflect(body1.velocity, normal);
			body2.velocity = glm::reflect(body2.velocity, normal);

			(*transform1.tHandle)->SetTranslation((*transform1.tHandle)->GetLocalTranslation() + body1.velocity * 0.02f);
			(*transform2.tHandle)->SetTranslation((*transform2.tHandle)->GetLocalTranslation() + body2.velocity * 0.02f);

			HandleColorCollision(e1, e2);
		}
	}

	void HandleColorCollision(entt::entity e1, entt::entity e2)
	{
		auto &color1 = ecsHandler->GetComponent<ColorComponent>(e1);
		auto &color2 = ecsHandler->GetComponent<ColorComponent>(e2);

		if (color1.color == color2.color)
			return;
		else if (color1.color == RED and color2.color == GREEN
			or color1.color == GREEN and color2.color == RED)
			{
				// Set both to RED
				color1.material->SetDiffuseColor(getColor(RED));
				color2.material->SetDiffuseColor(getColor(RED));
			}
		else if (color1.color == GREEN and color2.color == BLUE
			or color1.color == BLUE and color2.color == GREEN)
			{
				// Set both to GREEN
				color1.material->SetDiffuseColor(getColor(GREEN));
				color2.material->SetDiffuseColor(getColor(GREEN));
			}
		else if (color1.color == BLUE and color2.color == RED
			or color1.color == RED and color2.color == BLUE)
			{
				// Set both to BLUE
				color1.material->SetDiffuseColor(getColor(BLUE));
				color2.material->SetDiffuseColor(getColor(BLUE));
			}
	}

private:
	MonaECS::ECSHandler* ecsHandler;
	MonaECS::StatsSystem statsSystem;
	std::vector<entt::entity> walls;
	float time = 0.0f;
};
int main()
{
	ECSExample app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
}