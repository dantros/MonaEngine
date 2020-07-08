#pragma once
#ifndef GAMEOBJECT_HPP
#define GAMEOBJECT_HPP
#include <cstdint>
#include "../Core/Log.hpp"
#include <limits>
namespace Mona {

	using GameObjectID = uint32_t;
	constexpr GameObjectID INVALID_INDEX = 0;
	class World;
	template <typename ComponentType>
	class ComponentHandle;

	class GameObject {
	public:
		GameObject(){
			m_id = CreateNewObjectIndex();
		}
		virtual void Update(float timeStep) noexcept {};
		virtual void Start() noexcept {};
		GameObjectID GetObjectID() const noexcept{ return m_id; }

	private:
		GameObjectID m_id;

		static GameObjectID CreateNewObjectIndex() {

			static GameObjectID currentIndex = 0;
			//MONA_ASSERT(currentIndex < std::numeric_limits<GameObjectID>::max(), "WORLD: Cannot create another GameObject, maximun number of them has been reached");
			return ++currentIndex;
		}

	};
}
#endif