#include "MonaEngine.hpp"
#include <memory>
class Example1 : public Mona::Application
{
public:
	virtual void StartUp() noexcept override
	{
		Mona::Engine& engine = Mona::Engine::GetInstance();
		Mona::World &world = engine.GetWorld();
		//Helper function that returns an entityhandle with a StaticMeshComponent
		m_boxEntity = Mona::LoadModel(world, Mona::BasicPrimitives::Box);
		m_planeEntity = Mona::LoadModel(world, Mona::BasicPrimitives::Plane);
		m_box2Entity = world.CreateEntity();
		auto boxModel = engine.LoadModel(Mona::BasicPrimitves::Box);
		Mona::World::AddComponent<StaticMeshComponent>(m_box2Entity, boxModel);
	}
	virtual void Update(float timeStep) noexcept override
	{
		Mona::Input& input = Mona::Engine::GetInstance().GetInput();
		if (input.IsKeyPressed(MONA_KEY_ESCAPE))
		{
			SetApplicationShouldClose(true);
		}

	}
private:
	EntityHandle m_boxEntity;
	EntityHandle m_planeEntity;
	EntityHandle m_box2Entity;
};

int main()
{
	Mona::Engine& engine = Mona::Engine::GetInstance();
	engine.StartUp(std::make_unique<Example1>());
	engine.StartMainLoop();
	engine.ShutDown();

	return 0;
}