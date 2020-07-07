#pragma once
#ifndef GAMEOBJECTMANAGER_HPP
#define GAMEOBJECTMANAGER_HPP
#include "GameObject.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
namespace Mona {
	class GameObjectManager
	{
	public:
		GameObjectManager() {}
		void StartUp(GameObjectID expectedObjects = 0) noexcept;
		template <typename ObjectType,typename ...Args>
		std::weak_ptr<GameObject> CreateGameObject(Args&& ... args);
		std::weak_ptr<GameObject> GetGameObject(GameObjectID id) const noexcept;
		void UpdateGameObjects(float timeStep) noexcept;
		void DestroyGameObject(std::weak_ptr<GameObject> objectPointer) noexcept;
		void DestroyGameObject(GameObjectID id) noexcept;
	private:
		using GameObjectPtrs = std::vector<std::shared_ptr<GameObject>>;
		GameObjectPtrs m_gameObjects;
		std::unordered_map<GameObjectID, GameObjectPtrs::size_type> m_lookUp;
	};
}
#include "Detail/GameObjectManager.hpp"
#endif