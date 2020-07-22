#include "Core/Log.hpp"
#include "World/GameObject.hpp"
#include "World/World.hpp"
enum class State {
	Invalid,
	Started,
	Updated,
	ShuttedDown
};
int globalStartUpCalls = 0;
int globalShutDownCalls = 0;
int globalDestructorCalls = 0;
int globalUpdateCalls = 0;
class MyBox : public Mona::GameObject {
public:
	virtual void Update(Mona::World& world, float timeStep) noexcept override{
		globalUpdateCalls += 1;
		m_transform(world).Translate(glm::vec3(timeStep));
	};
	virtual void StartUp(Mona::World& world) noexcept override{
		globalStartUpCalls += 1;
		m_transform = AddComponent<Mona::TransformComponent>(world, *this);
		m_mesh = AddComponent<Mona::StaticMeshComponent>(world, *this);
		m_camera = AddComponent<Mona::CameraComponent>(world, *this);
	};
	virtual void ShutDown(Mona::World& world) noexcept override{
		globalShutDownCalls += 1;
	};
	glm::vec3 GetTranslation(Mona::World& world) const {
		return m_transform(world).GetLocalTranslation();
	}
	State GetBoxState() const {
		return m_boxState;
	}
	~MyBox() {
		globalDestructorCalls += 1;
	}
private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_mesh;
	Mona::CameraHandle m_camera;
	State m_boxState = State::Invalid;
};

Mona::GameObjectHandle<MyBox> boxes[2000];
int main(){
	Mona::Log::StartUp();
	Mona::EventManager eventManager;
	Mona::World world;
	world.StartUp(&eventManager, static_cast<Mona::GameObjectID>(1000));
	Mona::GameObjectHandle<MyBox> box = Mona::CreateGameObject<MyBox>(world);
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 1, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 1, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 1, "Incorrect component count");
	MONA_ASSERT(world.GetGameObjectCount() == 1, "Incorrect game object count");
	MONA_ASSERT(globalStartUpCalls == 1, "Incorrect number of startUp calls");
	world.Update(10.0f);
	world.Update(10.0f);
	MONA_ASSERT(globalUpdateCalls == 2, "Incorrect number of Update calls");
	MONA_ASSERT(box(world).GetTranslation(world) == glm::vec3(20.0f), "Incorrect box translation");
	Mona::DestroyGameObject(world, box);
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetGameObjectCount() == 0, "Incorrect game object count");
	MONA_ASSERT(globalShutDownCalls == 1, "Incorrect number of startUp calls");
	MONA_ASSERT(box.IsValid(world) == false, "box should be invalid");
	for (uint32_t i = 0; i < 2000; i++)
	{
		boxes[i] = Mona::CreateGameObject<MyBox>(world);
	}
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 2000, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 2000, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 2000, "Incorrect component count");
	MONA_ASSERT(world.GetGameObjectCount() == 2000, "Incorrect game object count");
	MONA_ASSERT(globalStartUpCalls == 2001, "Incorrect number of startUp calls");
	for (uint32_t i = 0; i < 100; i++)
	{
		world.Update(1.0f);
	}
	MONA_ASSERT(globalUpdateCalls == (2 + 2000*100), "Incorrect number of Update calls");
	MONA_ASSERT(globalShutDownCalls == globalDestructorCalls, "ShutDown calls must be equal to destructor calls");
	world.ShutDown();
	MONA_ASSERT(globalDestructorCalls == 2001, "Incorrect number of ShutDown calls");
	MONA_ASSERT(globalShutDownCalls == 1 , "ShutDown calls must be equal to destructor calls");
	MONA_LOG_INFO("All test passed!!!");
	return 0;
}