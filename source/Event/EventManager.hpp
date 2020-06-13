#pragma once
#ifndef EVENTMANAGER_HPP
#define EVENTMANAGER_HPP
#include <functional>
#include "Events.hpp"
#include <vector>
#include <unordered_map>
#include <typeindex>
namespace Mona
{
	class Engine;
	class EventManager {
		friend class Engine;
	public:
		template <typename ObjType, typename EventType>
		void Subscribe(ObjType* obj, void (ObjType::* memberFunction)(const EventType&)) {
			m_observers[std::type_index(typeid(EventType))].push_back([obj, memberFunction](const Event &e) {
				(obj->*memberFunction)(static_cast<const EventType&>(e)); });
		}

		template <typename EventType>
		void Publish(const EventType& e)
		{
			for (const auto& cb : m_observers[std::type_index(typeid(EventType))])
			{
				cb(e);
			}
		}

	private:
		EventManager() = default;
		~EventManager() = default;
		void ShutDown() noexcept;
		using callBack = std::function<void(const Event&)>;
		std::unordered_map<std::type_index, std::vector<callBack>> m_observers;

	};
}
#endif