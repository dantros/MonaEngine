#include "Core/Log.hpp"
#include "World/GameObject.hpp"
#include "World/World.hpp"
int globalStartUpCalls = 0;
int globalShutDownCalls = 0;
int globalDestructorCalls = 0;
int globalUpdateCalls = 0;
class MyBox : public Mona::GameObject {
public:
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override{
		globalUpdateCalls += 1;
		m_transform->Translate(glm::vec3(timeStep));
	};
	virtual void UserStartUp(Mona::World& world) noexcept override{
		globalStartUpCalls += 1;
		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_mesh = world.AddComponent<Mona::StaticMeshComponent>(*this);
		m_camera = world.AddComponent<Mona::CameraComponent>(*this);
	};
	virtual void UserShutDown(Mona::World& world) noexcept override{
		globalShutDownCalls += 1;
	};
	glm::vec3 GetTranslation() const {
		return m_transform->GetLocalTranslation();
	}
	~MyBox() {
		globalDestructorCalls += 1;
	}
private:
	Mona::TransformHandle m_transform;
	Mona::StaticMeshHandle m_mesh;
	Mona::CameraHandle m_camera;
};
class SpawnInShutDown : public Mona::GameObject {
public:
	virtual void UserStartUp(Mona::World& world) noexcept override {
		globalStartUpCalls += 1;
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		globalUpdateCalls += 1;
	}
	virtual void UserShutDown(Mona::World& world) noexcept override {
		globalShutDownCalls += 1;
	}
	~SpawnInShutDown() {
		globalDestructorCalls += 1;
	}
};
class FrameCountedObject : public Mona::GameObject {
public:
	virtual void UserStartUp(Mona::World& world) noexcept override {
		globalStartUpCalls += 1;
		m_frameCount = 0;
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		globalUpdateCalls += 1;
		m_frameCount += 1;
		if (m_frameCount == 1000)
		{
			world.DestroyGameObject(*this);
		}
	}
	virtual void UserShutDown(Mona::World& world) noexcept override {
		globalShutDownCalls += 1;
		world.CreateGameObject<SpawnInShutDown>();
	}

	~FrameCountedObject() {
		globalDestructorCalls += 1;
	}
private:
	int m_frameCount;
};

Mona::GameObjectHandle<MyBox> boxes[2000];
int main(){
	Mona::Log::StartUp();
	Mona::EventManager eventManager;
	Mona::World world;
	world.StartUp(&eventManager, static_cast<Mona::GameObjectID>(1000));
	Mona::GameObjectHandle<MyBox> box = world.CreateGameObject<MyBox>();
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetGameObjectCount() == 1, "Incorrect game object count");
	MONA_ASSERT(globalStartUpCalls == 0, "Incorrect number of startUp calls");
	world.Update(10.0f);
	world.Update(10.0f);
	MONA_ASSERT(globalStartUpCalls == 1, "Incorrect number of startUp calls");
	MONA_ASSERT(globalUpdateCalls == 2, "Incorrect number of Update calls");
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 1, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 1, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 1, "Incorrect component count");
	MONA_ASSERT(box->GetTranslation() == glm::vec3(20.0f), "Incorrect box translation");
	world.DestroyGameObject(box);
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 0, "Incorrect component count");
	MONA_ASSERT(world.GetGameObjectCount() == 0, "Incorrect game object count");
	MONA_ASSERT(globalShutDownCalls == 1, "Incorrect number of startUp calls");
	MONA_ASSERT(box.IsValid(world) == false, "box should be invalid");
	for (uint32_t i = 0; i < 2000; i++)
	{
		boxes[i] = world.CreateGameObject<MyBox>();
	}

	MONA_ASSERT(world.GetGameObjectCount() == 2000, "Incorrect game object count");
	for (uint32_t i = 0; i < 100; i++)
	{
		world.Update(1.0f);
	}
	MONA_ASSERT(world.GetComponentCount<Mona::TransformComponent>() == 2000, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::StaticMeshComponent>() == 2000, "Incorrect component count");
	MONA_ASSERT(world.GetComponentCount<Mona::CameraComponent>() == 2000, "Incorrect component count");
	MONA_ASSERT(globalStartUpCalls == 2001, "Incorrect number of startUp calls");
	MONA_ASSERT(globalUpdateCalls == (2 + 2000*100), "Incorrect number of Update calls");
	MONA_ASSERT(globalShutDownCalls == globalDestructorCalls, "ShutDown calls must be equal to destructor calls");
	Mona::GameObjectHandle<FrameCountedObject> testObject = world.CreateGameObject<FrameCountedObject>();
	for (uint32_t i = 0; i < 1000; i++)
	{
		world.Update(1.0f);
	}
	MONA_ASSERT(globalShutDownCalls == 2, "Incorrect number of startUp calls");
	MONA_ASSERT(globalStartUpCalls == 2002, "Incorrect number of startUp calls");
	world.Update(1.0f);
	MONA_ASSERT(globalStartUpCalls == 2003, "Incorrect number of startUp calls");
	world.ShutDown();
	MONA_ASSERT(globalDestructorCalls == 2003, "Incorrect number of ShutDown calls");
	MONA_LOG_INFO("All test passed!!!");
	return 0;
}