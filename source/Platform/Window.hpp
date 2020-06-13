#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP
#include <memory>
#include <glm/glm.hpp>
namespace Mona
{
	class Engine;
	class EventManager;
	class Window {
	public:
		friend class Engine;
		Window(const Window& window) = delete;
		Window& operator=(const Window& window) = delete;
		bool IsFullScreen() const noexcept;
		void SetFullScreen(bool value) noexcept;
		bool ShouldClose() const noexcept;
		void SetSwapInterval(int interval) noexcept;
		glm::ivec2 GetWindowDimensions() const noexcept;
		glm::ivec2 GetWindowFrameBufferSize() const noexcept;
		void SetWindowDimensions(const glm::ivec2 &dimensions) noexcept;
	private:
		Window();
		~Window();
		void StartUp(EventManager* eventManager) noexcept;
		void ShutDown() noexcept;
		void Update() noexcept;
		class Impl;
		std::unique_ptr<Impl> p_Impl;
	};
}
#endif