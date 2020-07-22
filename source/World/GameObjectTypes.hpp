#pragma once
#ifndef GAMEOBJECTTYPES_HPP
#define GAMEOBJECTTYPES_HPP
#include <limits>
#include <cstdint>
namespace Mona {
	using GameObjectID = uint32_t;
	constexpr GameObjectID INVALID_INDEX = std::numeric_limits<GameObjectID>::max();
	struct InnerComponentHandle
	{
		using size_type = GameObjectID;
		InnerComponentHandle() : m_index(INVALID_INDEX), m_generation(0) {};
		InnerComponentHandle(size_type index, size_type generation) :
			m_index(index),
			m_generation(generation) {};
		size_type m_index;
		size_type m_generation;
	};

	struct InnerGameObjectHandle
	{
		using size_type = GameObjectID;
		InnerGameObjectHandle() : m_index(INVALID_INDEX), m_generation(0) {};
		InnerGameObjectHandle(size_type index, size_type generation) :
			m_index(index),
			m_generation(generation) {};
		size_type m_index;
		size_type m_generation;
	};

}
#endif