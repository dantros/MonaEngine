#include "World.hpp"
namespace Mona {

	void World::Update(float timeStep) noexcept
	{
		m_objectManager.UpdateGameObjects(timeStep);
	}

}