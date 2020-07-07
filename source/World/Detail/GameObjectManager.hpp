#pragma once
#ifndef GAMEOBJECTMANAGER_DETAIL_HPP
#define GAMEOBJECTMANAGER_DETAIL_HPP
namespace Mona {
	template <typename ObjectType, typename ...Args>
	inline std::weak_ptr<GameObject> GameObjectManager::CreateGameObject(Args&& ... args) {
		static_assert(std::is_base_of<GameObject, ObjectType>::value, "ObjectType must be a derived class from GameObject");
		auto gameObjectPointer = std::make_shared<ObjectType>(std::forward<Args>(args)...);
		gameObjectPointer->Start();
		m_gameObjects.emplace_back(gameObjectPointer);
		m_lookUp[gameObjectPointer->GetObjectID()] = m_gameObjects.size() - 1;
		return std::weak_ptr<GameObject>(gameObjectPointer);
	}

}

#endif