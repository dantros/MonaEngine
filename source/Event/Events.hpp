#pragma once
#ifndef EVENTS_HPP
#define EVENTS_HPP
#include "../World/GameObject.hpp"
#include <cstdint>
#include "../Core/Common.hpp"
namespace Mona
{
	enum class EEventType : uint8_t {
		WindowResizeEvent,
		MouseScrollEvent,
		GameObjectDestroyedEvent,
		AplicationEndEvent,
		DebugGUIEvent,
		EventTypeCount
	};

	constexpr uint8_t GetEventIndex(EEventType type) {
		return static_cast<uint8_t>(type);
	}
	constexpr uint8_t GetEventTypeCount()
	{
		return static_cast<uint8_t>(EEventType::EventTypeCount);
	}
	struct Event {

	};
	constexpr uint32_t INVALID_EVENT_INDEX = std::numeric_limits<uint32_t>::max();
	class SubscriptionHandle {
	public:
		SubscriptionHandle() : m_index(INVALID_EVENT_INDEX), m_generation(0), m_typeIndex(GetEventTypeCount()) {};
		SubscriptionHandle(uint32_t index, uint32_t generation, uint8_t typeIndex) :
			m_index(index),
			m_generation(generation),
			m_typeIndex(typeIndex)
		{};
		friend class ObserverList;
		friend class EventManager;
	private:
		uint32_t m_index;
		uint32_t m_generation;
		uint8_t m_typeIndex;
	};
	struct WindowResizeEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::WindowResizeEvent);
		int width;
		int height;
	};

	struct MouseScrollEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::MouseScrollEvent);
		double xOffset;
		double yOffset;
	};

	struct GameObjectDestroyedEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::GameObjectDestroyedEvent);
		GameObjectDestroyedEvent(GameObject& go) : gameObject(go) {}
		GameObject& gameObject;
	};

	struct ApplicationEndEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::AplicationEndEvent);
	};

	struct DebugGUIEvent : public Event {
		static constexpr uint8_t eventIndex = GetEventIndex(EEventType::DebugGUIEvent);
	};

	template <typename EventType>
	inline constexpr bool is_event = is_any<EventType,
											WindowResizeEvent,
											MouseScrollEvent,
											GameObjectDestroyedEvent,
											ApplicationEndEvent,
											DebugGUIEvent>;

}
#endif