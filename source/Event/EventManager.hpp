#pragma once
#ifndef EVENTMANAGER_HPP
#define EVENTMANAGER_HPP
#include "../Core/Log.hpp"
#include "Events.hpp"
#include <functional>
#include <vector>
#include <unordered_map>
#include <typeindex>
#include <type_traits>
#include <array>
#include <limits>
namespace Mona
{
	class Engine;
	class ObserverList {
	public:
		ObserverList();
		using EventHandler = std::function<void(const Event&)>;
		static constexpr uint32_t s_maxEntries = INVALID_EVENT_INDEX;
		static constexpr uint32_t s_minFreeIndices = 10;
		SubscriptionHandle Subscribe(EventHandler handler, uint8_t typeIndex) noexcept {
			MONA_ASSERT(m_eventHandlers.size() < s_maxEntries, "EventManager Error: Cannot Add more observers, max number reached.");
			if (m_firstFreeIndex != s_maxEntries && m_freeIndicesCount > s_minFreeIndices)
			{
				auto& handleEntry = m_handleEntries[m_firstFreeIndex];
				MONA_ASSERT(handleEntry.active == false, "EventManager Error: Incorrect active state for handleEntry");
				MONA_ASSERT(handleEntry.generation < std::numeric_limits<decltype(handleEntry.generation)>::max(),
					"EventManager Error: Generational Index reached its maximunn value, observer cannot be added.");
				auto handleIndex = m_firstFreeIndex;

				if (m_firstFreeIndex == m_lastFreeIndex)
					m_firstFreeIndex = m_lastFreeIndex = s_maxEntries;
				else
					m_firstFreeIndex = handleEntry.index;
				handleEntry.generation += 1;
				handleEntry.active = true;
				handleEntry.index = static_cast<uint32_t>(m_eventHandlers.size());
				handleEntry.prevIndex = s_maxEntries;
				--m_freeIndicesCount;
				SubscriptionHandle resultHandle(handleIndex, handleEntry.generation, typeIndex);
				m_eventHandlers.push_back(handler);
				m_handleEntryIndices.emplace_back(handleIndex);
				return resultHandle;
			}
			else {
				m_handleEntries.emplace_back(static_cast<uint32_t>(m_eventHandlers.size()), s_maxEntries, 0);

				SubscriptionHandle resultHandle(static_cast<uint32_t>(m_handleEntries.size() - 1), 0, typeIndex);
				m_eventHandlers.push_back(handler);
				m_handleEntryIndices.emplace_back(static_cast<uint32_t>(m_handleEntries.size() - 1));
				return resultHandle;
			}
		}
		void Unsubscribe(const SubscriptionHandle& handle) noexcept;
		void Publish(const Event& e) noexcept;
		void ShutDown() noexcept;
	private:


		struct HandleEntry {
			HandleEntry(uint32_t i, uint32_t p, uint32_t g) : index(i), prevIndex(p), generation(g), active(true) {}
			uint32_t index;
			uint32_t prevIndex;
			uint32_t generation;
			bool active;
		};
		std::vector<HandleEntry> m_handleEntries;
		std::vector<uint32_t> m_handleEntryIndices;
		std::vector<EventHandler> m_eventHandlers;
		uint32_t m_firstFreeIndex;
		uint32_t m_lastFreeIndex;
		uint32_t m_freeIndicesCount;
		
	};


	class EventManager {
	public:
		template <typename ObjType, typename EventType>
		SubscriptionHandle Subscribe(ObjType* obj, void (ObjType::* memberFunction)(const EventType&)) {
			static_assert(is_event<EventType>, "Template parameter is not an event");
			auto eventHandler = [obj, memberFunction](const Event& e) { (obj->*memberFunction)(static_cast<const EventType&>(e)); };
			return m_observerLists[EventType::eventIndex].Subscribe(eventHandler, EventType::eventIndex);
		}
		
		template <typename EventType>
		SubscriptionHandle Subscribe(void (*freeFunction)(const EventType&)) {
			static_assert(is_event<EventType>, "Template parameter is not an event");
			auto eventHandler = [freeFunction](const Event& e) { (*freeFunction)(static_cast<const EventType&>(e)); };
			return m_observerLists[EventType::eventIndex].Subscribe(eventHandler, EventType::eventIndex);
		}

		template <typename EventType>
		void Publish(const EventType& e)
		{
			static_assert(is_event<EventType>, "Template parameter is not an event");
			m_observerLists[EventType::eventIndex].Publish(e);
		}
		
		void Unsubscribe(const SubscriptionHandle& handle) {
			m_observerLists[handle.m_typeIndex].Unsubscribe(handle);
		}
		EventManager() = default;
		~EventManager() = default;
		void ShutDown() noexcept;
	private:
		std::array<ObserverList, GetEventTypeCount()> m_observerLists;

	};
}
#endif