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
#include "../PhysicsCollision/PhysicsCollisionEvents.hpp"
namespace Mona
{
	class SubscriptionHandle {
	public:
		SubscriptionHandle(SubscriptionHandle const&) = delete;
		SubscriptionHandle& operator=(SubscriptionHandle const&) = delete;
		SubscriptionHandle() : m_index(INVALID_EVENT_INDEX), m_generation(0), m_typeIndex(GetEventTypeCount()) {};
		SubscriptionHandle(uint32_t index, uint32_t generation, uint8_t typeIndex) :
			m_index(index),
			m_generation(generation),
			m_typeIndex(typeIndex)
		{};
		~SubscriptionHandle();
		friend class ObserverList;
		friend class EventManager;
	private:
		void SetEventManager(EventManager* em) { m_eventManager = em; }
		uint32_t m_index;
		uint32_t m_generation;
		uint8_t m_typeIndex;
		EventManager* m_eventManager = nullptr;
	};

	class Engine;
	class ObserverList {
	public:
		ObserverList();
		using EventHandler = std::function<void(const Event&)>;
		static constexpr uint32_t s_maxEntries = INVALID_EVENT_INDEX;
		static constexpr uint32_t s_minFreeIndices = 10;
		void Subscribe(SubscriptionHandle& handle, EventHandler handler, uint8_t typeIndex) noexcept {
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
				//SubscriptionHandle resultHandle(handleIndex, handleEntry.generation, typeIndex);
				handle.m_index = handleIndex;
				handle.m_generation = handleEntry.generation;
				handle.m_typeIndex = typeIndex;
				m_eventHandlers.push_back(handler);
				m_handleEntryIndices.emplace_back(handleIndex);
				//return resultHandle;
			}
			else {
				m_handleEntries.emplace_back(static_cast<uint32_t>(m_eventHandlers.size()), s_maxEntries, 0);

				//SubscriptionHandle resultHandle(static_cast<uint32_t>(m_handleEntries.size() - 1), 0, typeIndex);
				handle.m_index = static_cast<uint32_t>(m_handleEntries.size() - 1);
				handle.m_generation = 0;
				handle.m_typeIndex = typeIndex;
				m_eventHandlers.push_back(handler);
				m_handleEntryIndices.emplace_back(static_cast<uint32_t>(m_handleEntries.size() - 1));
				//return resultHandle;
			}
		}
		bool IsSubcriptionHandleValid(const SubscriptionHandle& handle) {
			auto index = handle.m_index;
			if (index < m_handleEntries.size() &&
				handle.m_generation == m_handleEntries[index].generation &&
				m_handleEntries[index].active == true)
			{
				return true;
			}

			return false;
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
		void Subscribe(SubscriptionHandle& handle, ObjType* obj, void (ObjType::* memberFunction)(const EventType&)) {
			static_assert(is_event<EventType>, "Template parameter is not an event");
			auto eventHandler = [obj, memberFunction](const Event& e) { (obj->*memberFunction)(static_cast<const EventType&>(e)); };
			m_observerLists[EventType::eventIndex].Subscribe(handle, eventHandler, EventType::eventIndex);
			handle.SetEventManager(this);
			return;
		}
		
		template <typename EventType>
		void Subscribe(SubscriptionHandle& handle, void (*freeFunction)(const EventType&)) {
			static_assert(is_event<EventType>, "Template parameter is not an event");
			auto eventHandler = [freeFunction](const Event& e) { (*freeFunction)(static_cast<const EventType&>(e)); };
			m_observerLists[EventType::eventIndex].Subscribe(handle, eventHandler, EventType::eventIndex);
			handle.SetEventManager(this);
			return;
		}

		template <typename EventType>
		void Publish(const EventType& e)
		{
			static_assert(is_event<EventType>, "Template parameter is not an event");
			m_observerLists[EventType::eventIndex].Publish(e);
		}
		
		void Unsubscribe(SubscriptionHandle& handle) {
			MONA_ASSERT(handle.m_typeIndex < GetEventTypeCount(), "EventManager Error: Handle with invalid type index");
			m_observerLists[handle.m_typeIndex].Unsubscribe(handle);
			handle.SetEventManager(nullptr);
		}

		bool IsSubcriptionHandleValid(const SubscriptionHandle& handle) {
			if (handle.m_typeIndex >= GetEventTypeCount())
				return false;
			return m_observerLists[handle.m_typeIndex].IsSubcriptionHandleValid(handle);
		}
		EventManager() = default;
		~EventManager() = default;
		void ShutDown() noexcept;
	private:
		std::array<ObserverList, GetEventTypeCount()> m_observerLists;

	};
}
#endif