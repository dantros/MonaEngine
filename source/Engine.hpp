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

		/*
		* Función que necesita ser llamada una unica vez para iniciar correctamente el motor.
		*/
		void StartUp(std::unique_ptr<Application> app) noexcept;
		/*
		* Función que debe ser llamada para un correcto cierre del motor.
		*/
		void ShutDown() noexcept;
		/*
		* Función que comienza el main loop del motor, esta funcion debe ser llamada despues de inicializar el motor.
		*/
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