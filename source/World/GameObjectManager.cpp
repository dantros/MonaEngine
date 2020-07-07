#include "GameObjectManager.hpp"
#include "../Core/Log.hpp"
namespace Mona {
	void GameObjectManager::StartUp(GameObjectID expectedObjects) noexcept
	{
		m_gameObjects.reserve(expectedObjects);
		m_lookUp.reserve(expectedObjects);
	}
	std::weak_ptr<GameObject> GameObjectManager::GetGameObject(GameObjectID id) const noexcept
	{
		auto it = m_lookUp.find(id);
		if (it != m_lookUp.end())
		{
			return std::weak_ptr<GameObject>(m_gameObjects[it->second]);
		}
		return std::weak_ptr<GameObject>();
	}

	void GameObjectManager::UpdateGameObjects(float timeStep) noexcept {
		for (auto& objectPtr : m_gameObjects)
		{
			objectPtr->Update(timeStep);
		}
	}
	void GameObjectManager::DestroyGameObject(GameObjectID id) noexcept
	{
		auto it = m_lookUp.find(id);
		if (it != m_lookUp.end())
		{
			auto index = it->second;
			if (index < m_gameObjects.size() - 1)
			{
				m_gameObjects[index] = std::move(m_gameObjects.back());
				m_lookUp[m_gameObjects[index]->GetObjectID()] = index;
			}
			m_gameObjects.pop_back();
			m_lookUp.erase(it);
		}
		return;
	}
	void GameObjectManager::DestroyGameObject(std::weak_ptr<GameObject> objectPointer) noexcept
	{
		auto sharedObjectPtr = objectPointer.lock();

		if (sharedObjectPtr != nullptr)
		{
			auto id = sharedObjectPtr->GetObjectID();
			DestroyGameObject(id);
			return;
		}
		MONA_LOG_ERROR("WORLD: Attempting to destroy GameObject from invalid weak_ptr");
	}
}