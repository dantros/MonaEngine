#pragma once
#ifndef EVENTS_HPP
#define EVENTS_HPP
#include "../World/GameObject.hpp"
#include <cstdint>
namespace Mona
{
	enum class EventType : uint8_t {
		WindowResizeEvent,
		MouseScrollEvent,
		GameObjectDestroyedEvent,
		EventTypeCount
	};

	constexpr uint8_t GetEventIndex(EventType type) {
		return static_cast<uint8_t>(type);
	}
	constexpr uint8_t GetEventTypeCount()
	{
		return static_cast<uint8_t>(EventType::EventTypeCount);
	}
	struct Event {

	};
	constexpr uint32_t INVALID_EVENT_INDEX = std::numeric_limits<uint32_t>::max();
	class SubscriptionHandle {
	public:
		SubscriptionHandle() : m_index(INVALID_EVENT_INDEX), m_generation(0) {};
		SubscriptionHandle(uint32_t index, uint32_t generation) :
			m_index(index),
			m_generation(generation) {};
		friend class ObserverList;
	private:
		uint32_t m_index;
		uint32_t m_generation;
	};
	struct WindowResizeEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EventType::WindowResizeEvent);
		int width;
		int height;
	};

	struct MouseScrollEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EventType::MouseScrollEvent);
		double xOffset;
		double yOffset;
	};

	struct GameObjectDestroyedEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EventType::GameObjectDestroyedEvent);
		GameObjectDestroyedEvent(GameObject& go) : gameObject(go) {}
		GameObject& gameObject;
	};


}
#endif