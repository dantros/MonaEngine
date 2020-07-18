#pragma once
#ifndef GAMEOBJECTTYPES_HPP
#define GAMEOBJECTTYPES_HPP
#include <limits>
#include <cstdint>
namespace Mona {
	using GameObjectID = uint32_t;
	constexpr GameObjectID INVALID_INDEX = std::numeric_limits<GameObjectID>::max();
	struct ComponentHandle
	{
		using size_type = GameObjectID;
		ComponentHandle() : m_index(INVALID_INDEX), m_generation(0) {};
		ComponentHandle(size_type index, size_type generation) :
			m_index(index),
			m_generation(generation) {};
		size_type m_index;
		size_type m_generation;
	};

	struct GameObjectHandle
	{
		using size_type = GameObjectID;
		GameObjectHandle() : m_index(INVALID_INDEX), m_generation(0) {};
		GameObjectHandle(size_type index, size_type generation) :
			m_index(index),
			m_generation(generation) {};
		size_type m_index;
		size_type m_generation;
	};

}
#endif