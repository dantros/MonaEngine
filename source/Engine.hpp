#pragma once
#ifndef ENGINE_HPP
#define ENGINE_HPP
#include "Platform/Window.hpp"
#include "Platform/Input.hpp"
#include "Event/EventManager.hpp"
namespace Mona {
	class Engine
	{
	public:
		Engine() noexcept : m_window(), m_input(), m_eventManager() {}
		void StartUp() noexcept;
		void ShutDown() noexcept;
		void StartMainLoop() noexcept;
		EventManager& GetEventManager() { return m_eventManager; }
		Window& GetWindow() { return m_window; }
		Input& GetInput() { return m_input; }
	private:
		Input m_input;
		Window m_window;
		EventManager  m_eventManager;
	};
}

#endif