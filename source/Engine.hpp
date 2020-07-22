#pragma once
#ifndef ENGINE_HPP
#define ENGINE_HPP
#include "Platform/Window.hpp"
#include "Platform/Input.hpp"
#include "Event/EventManager.hpp"
#include "Application.hpp"
#include "World/World.hpp"
#include <memory>
namespace Mona {
	class Engine
	{
	public:
		Engine(const Engine&) = delete;
		Engine& operator=(const Engine&) = delete;
		void StartUp(std::unique_ptr<Application> app) noexcept;
		void ShutDown() noexcept;
		void StartMainLoop() noexcept;
		static Engine& GetInstance() noexcept {
			static Engine s_engine;
			return s_engine;
		}
		EventManager& GetEventManager() noexcept{ return m_eventManager; }
	private:
		Engine() = default;
		~Engine() = default;
		EventManager  m_eventManager;
		World m_world;
		std::shared_ptr<Window> m_window;
		std::shared_ptr<Input> m_input;
		std::unique_ptr<Application> m_application;
	};
}

#endif