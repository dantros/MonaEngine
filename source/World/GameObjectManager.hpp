#pragma once
#ifndef GAMEOBJECTMANAGER_HPP
#define GAMEOBJECTMANAGER_HPP
#include "GameObject.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
namespace Mona {
	class World;
	class GameObjectManager
	{
	public:
		
		using size_type = GameObjectID;
		GameObjectManager();
		GameObjectManager(const GameObjectManager& manager) = delete;
		GameObjectManager& operator=(const GameObjectManager& manager) = delete;
		void StartUp(GameObjectID expectedObjects = 0) noexcept;
		void ShutDown(World &world) noexcept;
		template <typename ObjectType,typename ...Args>
		InnerGameObjectHandle CreateGameObject(World &world, Args&& ... args);
		void DestroyGameObject(World& world, const InnerGameObjectHandle &handle) noexcept;
		
		GameObject& GetGameObjectReference(const InnerGameObjectHandle& handle) noexcept;
		size_type GetCount() const noexcept;
		bool IsValid(const InnerGameObjectHandle& handle) const noexcept;


		void UpdateGameObjects(World& world, float timeStep) noexcept;
	private:
		
		//using GameObjectPtrs = std::vector<std::shared_ptr<GameObject>>;
		//GameObjectPtrs m_gameObjects;
		//std::unordered_map<GameObjectID, GameObjectPtrs::size_type> m_lookUp;
		//std::unordered_map<GameObjectID, GameObjectPtrs::size_type> m_lookUp;
		constexpr static size_type s_maxEntries = std::numeric_limits<size_type>::max();
		constexpr static size_type s_minFreeIndices = 1024;
		struct HandleEntry {
			HandleEntry(size_type i, size_type p, size_type g) : index(i), prevIndex(p), generation(g), active(true) {}
			size_type index;
			size_type prevIndex;
			size_type generation;
			bool active;
		};
		std::vector<std::unique_ptr<GameObject>> m_gameObjects;
		std::vector<size_type> m_gameObjectHandleIndices;
		std::vector<HandleEntry> m_handleEntries;

		std::vector<std::unique_ptr<GameObject>> m_pendingAddObjects;
		std::vector<InnerGameObjectHandle> m_pendingDestroyObjectHandles;
		size_type m_firstFreeIndex;
		size_type m_lastFreeIndex;
		size_type m_freeIndicesCount;
		bool m_IsIteratingOverObject;
	};
}
#include "Detail/GameObjectManager_Implementation.hpp"
#endif