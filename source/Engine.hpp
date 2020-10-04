#pragma once
#ifndef ENGINE_HPP
#define ENGINE_HPP
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
	private:
		Engine() = default;
		~Engine() = default;
		World m_world;
	};
}

#endif