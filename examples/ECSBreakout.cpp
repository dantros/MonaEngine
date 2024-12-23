#include <unordered_map>
#include "MonaEngine.hpp"
#include "ECS/ECS.hpp"
#include "ECS/Systems.hpp"
#include "ECS/Components.hpp"
#include "ECS/Events.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"

void CreateBasicCameraWithMusicAndLight(Mona::World &world)
{
    auto camera = world.CreateGameObject<Mona::GameObject>();
    auto transform = world.AddComponent<Mona::TransformComponent>(camera, glm::vec3(0.0f, -15.0f, 15.0f));
    transform->Rotate(glm::vec3(-1.0f, 0.0f, 0.0f), 0.5f);
    world.SetMainCamera(world.AddComponent<Mona::CameraComponent>(camera));
    world.SetAudioListenerTransform(transform);
    auto &config = Mona::Config::GetInstance();
    auto &audioClipManager = Mona::AudioClipManager::GetInstance();
    auto audioClipPtr = audioClipManager.LoadAudioClip(config.getPathOfApplicationAsset("AudioFiles/music.wav"));
    auto audioSource = world.AddComponent<Mona::AudioSourceComponent>(camera, audioClipPtr);
    audioSource->SetIsLooping(true);
    audioSource->SetVolume(0.3f);
    audioSource->Play();

    world.AddComponent<Mona::DirectionalLightComponent>(camera, glm::vec3(1.0f));
}

class Wall : public Mona::GameObject
{
public:
    Wall(const glm::vec3 &position,
         const glm::vec3 &scale)
        : m_position(position), m_scale(scale) {};
    ~Wall() = default;
    virtual void UserStartUp(Mona::World &world) noexcept
    {
        MonaECS::ECSHandler *ecs = world.GetECSHandler();
        m_transform = world.AddComponent<Mona::TransformComponent>(*this);
        auto wall = world.CreateGameObject<Mona::GameObject>();
        auto &meshManager = Mona::MeshManager::GetInstance();

        m_wallTransform = world.AddComponent<Mona::TransformComponent>(wall);
        m_wallTransform->SetTranslation(m_position);
        m_wallTransform->SetScale(m_scale);

        auto wallMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
        world.AddComponent<Mona::StaticMeshComponent>(wall, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), wallMaterial);

        auto wallEntity = ecs->CreateEntity();
        ecs->AddComponent<MonaECS::TransformComponent>(wallEntity, &m_wallTransform);
        ecs->AddComponent<MonaECS::BodyComponent>(wallEntity, glm::vec3(0.0f), glm::vec3(0.0f), 10.0f);
        ecs->AddComponent<MonaECS::ColliderComponent>(wallEntity, m_scale, false);
    }

private:
    const glm::vec3 &m_position;
    const glm::vec3 &m_scale;
    Mona::TransformHandle m_transform;
    Mona::TransformHandle m_wallTransform;
};

class Block : public Mona::GameObject
{
public:
    Block(const glm::vec3 &position, const glm::vec3 &scale) : m_position(position), m_scale(scale) {};
    ~Block() = default;
    virtual void UserStartUp(Mona::World &world) noexcept
    {
        m_world = &world;
        ecs = world.GetECSHandler();
        m_transform = world.AddComponent<Mona::TransformComponent>(*this);
        auto block = world.CreateGameObject<Mona::GameObject>();
        auto &meshManager = Mona::MeshManager::GetInstance();

        m_blockTransform = world.AddComponent<Mona::TransformComponent>(block);
        m_blockTransform->SetTranslation(m_position);
        m_blockTransform->SetScale(m_scale);

        auto redBlockMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
        redBlockMaterial->SetDiffuseColor(glm::vec3(1.0f, 0.0f, 0.0f));
        auto greenBlockMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
        greenBlockMaterial->SetDiffuseColor(glm::vec3(0.0f, 1.0f, 0.0f));
        auto blueBlockMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
        blueBlockMaterial->SetDiffuseColor(glm::vec3(0.0f, 0.0f, 1.0f));

        m_materialVector.push_back(redBlockMaterial);
        m_materialVector.push_back(greenBlockMaterial);
        m_materialVector.push_back(blueBlockMaterial);

        m_meshHandle = world.AddComponent<Mona::StaticMeshComponent>(block, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), m_materialVector[0]);

        ent = ecs->CreateEntity();
        ecs->AddComponent<MonaECS::TransformComponent>(ent, &m_blockTransform);
        ecs->AddComponent<MonaECS::BodyComponent>(ent, glm::vec3(0.0f), glm::vec3(0.0f), 10.0f);
        ecs->AddComponent<MonaECS::ColliderComponent>(ent, m_scale, false);
        ecs->AddComponent<MonaECS::Stats>(ent);
        ecs->SubscribeEvent<MonaECS::ColorChangeEvent, Block, &Block::OnColorChange>(*this);
        ecs->SubscribeEvent<MonaECS::DestroyGameObjectEvent, Block, &Block::OnDestroyGameObjectEvent>(*this);
    }

    virtual void UserShutDown(Mona::World &world) noexcept {};

    void OnColorChange(const MonaECS::ColorChangeEvent &event)
    {
        if (ent == event.entity)
        {
            entt::registry &registry = ecs->GetRegistry();
            auto stat = registry.get<MonaECS::Stats>(ent);
            m_meshHandle->SetMaterial(m_materialVector[static_cast<int>(stat.state)]);
        }
    }
    void OnDestroyGameObjectEvent(const MonaECS::DestroyGameObjectEvent &event)
    {
        if (ent == event.entity)
        {
            ecs->UnsubscribeEvent<MonaECS::ColorChangeEvent, Block, &Block::OnColorChange>(*this);
            ecs->UnsubscribeEvent<MonaECS::DestroyGameObjectEvent, Block, &Block::OnDestroyGameObjectEvent>(*this);
            ecs->DestroyEntity(ent);
            m_blockTransform->Translate(glm::vec3(100.0f));
            m_world->DestroyGameObject(*this);
        }
    }

private:
    const glm::vec3 &m_position;
    const glm::vec3 &m_scale;
    Mona::TransformHandle m_transform;
    Mona::TransformHandle m_blockTransform;
    Mona::StaticMeshHandle m_meshHandle;
    Mona::World *m_world;
    MonaECS::ECSHandler *ecs;
    entt::entity ent;
    std::vector<std::shared_ptr<Mona::DiffuseFlatMaterial>> m_materialVector;
};

class Paddle : public Mona::GameObject
{
public:
    Paddle() {};
    ~Paddle() = default;
    virtual void UserStartUp(Mona::World &world) noexcept
    {
        MonaECS::ECSHandler *ecs = world.GetECSHandler();
        m_transform = world.AddComponent<Mona::TransformComponent>(*this);
        auto paddle = world.CreateGameObject<Mona::GameObject>();
        auto &meshManager = Mona::MeshManager::GetInstance();

        m_paddleTransform = world.AddComponent<Mona::TransformComponent>(paddle);
        m_paddleTransform->SetTranslation(m_transform->GetLocalTranslation());
        m_paddleTransform->SetScale(m_scale);

        auto paddleMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
        paddleMaterial->SetDiffuseColor(glm::vec3(0.2f, 0.2f, 0.2f));
        world.AddComponent<Mona::StaticMeshComponent>(paddle, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Cube), paddleMaterial);

        auto paddleEntity = ecs->CreateEntity();
        ecs->AddComponent<MonaECS::TransformComponent>(paddleEntity, &m_paddleTransform);
        ecs->AddComponent<MonaECS::BodyComponent>(paddleEntity, glm::vec3(0.0f), glm::vec3(0.0f), 10.0f);
        ecs->AddComponent<MonaECS::ColliderComponent>(paddleEntity, m_scale, false);
        ecs->AddComponent<MonaECS::MoveInputComponent>(paddleEntity);
        ecs->SubscribeEvent<MonaECS::MoveInputEvent, Paddle, &Paddle::OnInputEvent>(*this);
    }

private:
    void OnInputEvent(const MonaECS::MoveInputEvent &event)
    {
        if (event.moveDir.y == 0 && event.moveDir.z == 0)
        {
            glm::vec3 direction = event.moveDir * 0.1f;
            glm::vec3 pos = m_paddleTransform->GetLocalTranslation();
            m_paddleTransform->SetTranslation(pos + direction);
        }
    }

    const glm::vec3 &m_scale = glm::vec3(3.0f, 0.5f, 1.0f);
    Mona::TransformHandle m_transform;
    Mona::TransformHandle m_paddleTransform;
};

class Ball : public Mona::GameObject
{
public:
    Ball() {};
    ~Ball() = default;
    virtual void UserStartUp(Mona::World &world) noexcept
    {
        ecs = world.GetECSHandler();
        m_transform = world.AddComponent<Mona::TransformComponent>(*this);
        auto &meshManager = Mona::MeshManager::GetInstance();
        m_ballHandle = world.CreateGameObject<Mona::GameObject>();
        float ballRadius = 0.5f;

        m_ballTransform = world.AddComponent<Mona::TransformComponent>(m_ballHandle);
        m_ballTransform->SetRotation(m_transform->GetLocalRotation());
        m_ballTransform->SetTranslation(m_transform->GetLocalTranslation() + glm::vec3(0.0f, 2.0f, 0.0f));
        m_ballTransform->SetScale(glm::vec3(ballRadius));

        auto ballMaterial = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
        ballMaterial->SetDiffuseColor(glm::vec3(0.75f, 0.3f, 0.3f));
        world.AddComponent<Mona::StaticMeshComponent>(m_ballHandle, meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Sphere), ballMaterial);

        auto ballEntity = ecs->CreateEntity();
        ecs->AddComponent<MonaECS::TransformComponent>(ballEntity, &m_ballTransform);
        ecs->AddComponent<MonaECS::BodyComponent>(ballEntity, glm::vec3(2.0f, 10.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), 10.0f);
        ecs->AddComponent<MonaECS::ColliderComponent>(ballEntity, glm::vec3(ballRadius), false);
        ecs->SubscribeEvent<MonaECS::CollisionEvent, Ball, &Ball::OnCollision>(*this);
    }
    void OnCollision(const MonaECS::CollisionEvent &event)
    {
        auto e1 = event.entity1;
        auto e2 = event.entity2;
        auto normal = event.normal;
        entt::registry &registry = ecs->GetRegistry();

        auto &transform1 = ecs->GetComponent<MonaECS::TransformComponent>(e1);
        auto &transform2 = ecs->GetComponent<MonaECS::TransformComponent>(e2);

        auto &body1 = ecs->GetComponent<MonaECS::BodyComponent>(e1);
        auto &body2 = ecs->GetComponent<MonaECS::BodyComponent>(e2);

        if (registry.any_of<MonaECS::Stats>(e1))
        {
            auto &stats = ecs->GetComponent<MonaECS::Stats>(e1);
            stats.health -= 40;
        }
        else if (registry.any_of<MonaECS::Stats>(e2))
        {
            auto &stats = ecs->GetComponent<MonaECS::Stats>(e2);
            stats.health -= 40;
        }

        body1.velocity = glm::reflect(body1.velocity, normal);
        body2.velocity = glm::reflect(body2.velocity, normal);

        body1.acceleration = glm::reflect(body1.acceleration, normal);
        body2.acceleration = glm::reflect(body2.acceleration, normal);

        (*transform1.tHandle)->SetTranslation((*transform1.tHandle)->GetLocalTranslation() + body1.velocity * 0.02f);
        (*transform2.tHandle)->SetTranslation((*transform2.tHandle)->GetLocalTranslation() + body2.velocity * 0.02f);
    }

private:
    Mona::TransformHandle m_transform;
    Mona::TransformHandle m_ballTransform;
    Mona::RigidBodyHandle m_ballRigidBody;
    Mona::GameObjectHandle<Mona::GameObject> m_ballHandle;
    MonaECS::ECSHandler *ecs;
    std::shared_ptr<Mona::AudioClip> m_ballBounceSound;
};

class Breakout : public Mona::Application
{
public:
    Breakout() = default;
    ~Breakout() = default;
    virtual void UserStartUp(Mona::World &world) noexcept override
    {
        ecs = world.GetECSHandler();
        world.SetGravity(glm::vec3(0.0f, 0.0f, 0.0f));
        world.SetAmbientLight(glm::vec3(0.3f));
        CreateBasicCameraWithMusicAndLight(world);

        world.CreateGameObject<Ball>();

        world.CreateGameObject<Wall>(glm::vec3(0.0f, 26.0f, 0.0f), glm::vec3(18.0f, 1.0f, 1.0f));
        glm::vec3 sideWallScale(1.0f, 27.0f, 1.0f);
        float sideWallOffset = 19.0f;
        world.CreateGameObject<Wall>(glm::vec3(-sideWallOffset, 0.0f, 0.0f), sideWallScale);
        world.CreateGameObject<Wall>(glm::vec3(sideWallOffset, 0.0f, 0.0f), sideWallScale);

        world.CreateGameObject<Block>(glm::vec3(10.0f, 15.0f, 0.0f), glm::vec3(3.0f, 1.0f, 1.0f));
        world.CreateGameObject<Block>(glm::vec3(0.0f, 15.0f, 0.0f), glm::vec3(3.0f, 1.0f, 1.0f));
        world.CreateGameObject<Block>(glm::vec3(-10.0f, 15.0f, 0.0f), glm::vec3(3.0f, 1.0f, 1.0f));

        world.CreateGameObject<Paddle>();

        ecs->RegisterSystem<MonaECS::StatsSystem>();
        ecs->RegisterSystem<MonaECS::CollisionSystem>();
        ecs->RegisterSystem<MonaECS::MovementSystem>();
        ecs->RegisterSystem<MonaECS::InputSystem>();
        ecs->StartUpSystems();
    }

    virtual void UserShutDown(Mona::World &world) noexcept override
    {
        ecs->ShutDownSystems();
    }
    virtual void UserUpdate(Mona::World &world, float timeStep) noexcept override
    {
        ecs->UpdateSystems(timeStep);
    }
    std::shared_ptr<Mona::AudioClip> m_blockBreakingSound;

private:
    MonaECS::ECSHandler *ecs;
};
int main()
{
    Breakout breakout;
    Mona::Engine engine(breakout);
    engine.StartMainLoop();
}