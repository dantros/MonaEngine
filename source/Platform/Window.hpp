#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP
#include <memory>
#include <glm/glm.hpp>
namespace Mona
{
	class World;
	class EventManager;
	/*
	* La clase Window provee una interfaz que permite al usario configurar y/o consultar información de la ventana de la aplicación.
	*/
	class Window {
	public:
		friend class World;
		Window();
		~Window();
		Window(const Window& window) = delete;
		Window& operator=(const Window& window) = delete;

		/*
		* Retorna verdadero si la ventana de la aplicación esta en modo pantalla completa.
		*/
		bool IsFullScreen() const noexcept;

		/*
		* Ajusta 
		*/
		void SetFullScreen(bool value) noexcept;

		/*
		* 
		*/
		bool ShouldClose() const noexcept;

		/*
		* 
		*/
		void SetSwapInterval(int interval) noexcept;

		/*
		* 
		*/
		glm::ivec2 GetWindowDimensions() const noexcept;

		/*
		* 
		*/
		glm::ivec2 GetWindowFrameBufferSize() const noexcept;

		/*
		* 
		*/
		void SetWindowDimensions(const glm::ivec2 &dimensions) noexcept;
	private:
		void StartUp(EventManager& eventManager) noexcept;
		void ShutDown() noexcept;
		void Update() noexcept;
		class WindowImplementation;
		std::unique_ptr<WindowImplementation> p_Impl;
	};
}
#endif