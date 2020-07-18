#include <iostream>
#include <cassert>
#include "Core/Log.hpp"
#include "World/GameObjectManager.hpp"
void assert_message(bool value, const std::string& message)
{
	if (!value)
	{
		std::cout << message << "\n";
	}
	assert(value);
};

enum class State {
	Invalid,
	Started,
	Updated
};
class MyBox : public Mona::GameObject {
public:
	MyBox() = default;
	virtual void StartUp(Mona::World& world) noexcept override {
		boxState = State::Started;
	}
	virtual void Update(Mona::World& world, float timeStep) noexcept override {
		boxState = State::Updated;
	}
	State GetState() const { return boxState; }
private:
	State boxState = State::Invalid;
};
int main()
{
	Mona::Log::StartUp();
	std::cout << "All test passed!!!\n";
	return 0;
}
