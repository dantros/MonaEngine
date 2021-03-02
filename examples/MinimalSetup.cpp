#include "MonaEngine.hpp"
class MinimalSetup : public Mona::Application
{
public:
	MinimalSetup() = default;
	~MinimalSetup() = default;
	virtual void UserStartUp(Mona::World &world) noexcept override{
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
	}
};
int main()
{	
	MinimalSetup app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
	
}