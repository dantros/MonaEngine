#pragma once
#ifndef WINDOW_HPP
#define WINDOW_HPP
#include <memory>
#include <glm/glm.hpp>
namespace Mona
{
	class Window {
	public:
		Window();
		~Window();
		Window(const Window& window) = delete;
		Window& operator=(const Window& window) = delete;
		//TODO(Byron) Maybe move StartUp/ShutDown/Update to private and make caller friend class
		void StartUp() noexcept;
		void ShutDown() noexcept;
		void Update() noexcept;
		bool IsFullScreen() const noexcept;
		void SetFullScreen(bool value) noexcept;
		bool ShouldClose() const noexcept;
		void SetSwapInterval(int interval) noexcept;
		glm::ivec2 GetWindowDimensions() const noexcept;
		glm::ivec2 GetWindowFrameBufferSize() const noexcept;
		void SetWindowDimensions(const glm::ivec2 &dimensions) noexcept;
	private:
		class Impl;
		std::unique_ptr<Impl> p_Impl;
	};
}
#endif