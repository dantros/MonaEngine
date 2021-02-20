#include "Window.hpp"
#include "../Core/Config.hpp"
#include "../Core/Common.hpp"
#include "../Event/Events.hpp"
#include "../Event/EventManager.hpp"
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifndef _WINDOWS_
#undef APIENTRY
#endif

namespace Mona {
	static void GLFWErrorCallback(int error, const char* description)
	{
		MONA_LOG_ERROR("GLFW Error ({0}): {1}", error, description);
	}
	class Window::WindowImplementation {
	public:
		WindowImplementation() : m_data(){}
		WindowImplementation(const WindowImplementation& window) = delete;
		WindowImplementation& operator=(const WindowImplementation& window) = delete;
		void StartUp(EventManager& eventManager) noexcept
		{
			MONA_ASSERT(m_windowHandle == nullptr, "Calling Window::StartUp for the second time!!!.");
			const int success = glfwInit();
			MONA_ASSERT(success, "Could not initialize GLFW!");
			glfwSetErrorCallback(GLFWErrorCallback);
			Mona::Config& config = Config::GetInstance(); 
			const int glVersionMajor = config.getValueOrDefault<int>("OpenGL_major_version", 4);
			const int glVersionMinor = config.getValueOrDefault<int>("OpenGL_minor_version", 5);
			auto windowTitle = config.getValueOrDefault<std::string>("windowTitle", "Default Title");
			int windowWidth = config.getValueOrDefault<int>("windowWidth", 1440);
			int windowHeight = config.getValueOrDefault<int>("windowHeight", 810);
			bool fullScreen = config.getValueOrDefault<bool>("fullscreen", false);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, glVersionMajor);
			glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, glVersionMinor);
			glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if __APPLE__
			glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif
#ifndef NDEBUG
			glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);
#endif
			auto monitor = glfwGetPrimaryMonitor();
			m_windowHandle = glfwCreateWindow(windowWidth, windowHeight, windowTitle.c_str(), fullScreen? monitor : NULL, NULL);
			glfwMakeContextCurrent(m_windowHandle);
			glfwGetWindowPos(m_windowHandle, &m_oldWindowPos[0], &m_oldWindowPos[1]);
			m_data.eventManager = &eventManager;
			glfwSetWindowUserPointer(m_windowHandle, &m_data);
			
			//
			glfwSetFramebufferSizeCallback(m_windowHandle, [](GLFWwindow* window, int width, int height)
				{
					WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
					WindowResizeEvent e;
					e.width = width;
					e.height = height;
					(data.eventManager)->Publish(e);
				});
			glfwSetScrollCallback(m_windowHandle, [](GLFWwindow* window, double xOffset, double yOffset)
				{
					WindowData& data = *(WindowData*)glfwGetWindowUserPointer(window);
					MouseScrollEvent e;
					e.xOffset = xOffset;
					e.yOffset = yOffset;
					(data.eventManager)->Publish(e);
				});
			int status = gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
			MONA_ASSERT(status, "Failed to initialize Glad!");

			glfwSwapInterval(1);
		}
		void ShutDown() noexcept
		{
			MONA_ASSERT(m_windowHandle != nullptr, "Calling Window::ShutDown for the second time or without calling Window::Startup first.");
			glfwDestroyWindow(m_windowHandle);
			//glfwTerminate();
		}
		void Update() noexcept
		{
			glfwSwapBuffers(m_windowHandle);		
		}
		bool IsFullScreen() const noexcept
		{
			return glfwGetWindowMonitor(m_windowHandle) != NULL;
		}
		void SetFullScreen(bool value) noexcept
		{
			if (value == IsFullScreen())
				return;
			if (value)
			{
				glfwGetWindowPos(m_windowHandle, &m_oldWindowPos[0], &m_oldWindowPos[1]);
				auto monitor = glfwGetPrimaryMonitor();
				int width, height;
				glfwGetWindowSize(m_windowHandle, &width, &height);
				glfwSetWindowMonitor(m_windowHandle, monitor, 0, 0, width, height, GLFW_DONT_CARE);
			}
			else {
				int width, height;
				glfwGetWindowSize(m_windowHandle, &width, &height);
				glfwSetWindowMonitor(m_windowHandle, NULL, m_oldWindowPos.x, m_oldWindowPos.y, width, height, GLFW_DONT_CARE);
			}
			
			
		}
		bool ShouldClose() const noexcept
		{
			return glfwWindowShouldClose(m_windowHandle);
		}
		void SetSwapInterval(int interval) noexcept
		{
			glfwSwapInterval(interval);
		}
		glm::ivec2 GetWindowDimensions() const noexcept
		{
			int width, height;
			glfwGetWindowSize(m_windowHandle, &width, &height);
			return glm::ivec2(width,height);
		}
		glm::ivec2 GetWindowFrameBufferSize() const noexcept
		{
			int width, height;
			glfwGetFramebufferSize(m_windowHandle, &width, &height);
			return glm::ivec2(width, height);
		}
		void SetWindowDimensions(const glm::ivec2 &dimensions) noexcept
		{
			glfwSetWindowSize(m_windowHandle, dimensions.x, dimensions.y);
		}
		GLFWwindow* GetHandle() const {
			return m_windowHandle;
		}
	private:
		struct WindowData {
			EventManager* eventManager = nullptr;
		};
		GLFWwindow* m_windowHandle = nullptr;
		WindowData m_data;
		glm::ivec2 m_oldWindowPos = glm::vec2(0,0);
	};

	Window::Window() : p_Impl(std::make_unique<WindowImplementation>()) {}

	Window::~Window() = default;

	void Window::StartUp(EventManager& eventManager) noexcept
	{
		p_Impl->StartUp(eventManager);
	}

	void Window::ShutDown() noexcept
	{
		p_Impl->ShutDown();
	}

	void Window::Update() noexcept
	{
		p_Impl->Update();
	}

	bool Window::IsFullScreen() const noexcept
	{
		return p_Impl->IsFullScreen();
	}

	void Window::SetFullScreen(bool value) noexcept
	{
		p_Impl->SetFullScreen(value);
	}

	bool Window::ShouldClose() const noexcept
	{
		return p_Impl->ShouldClose();
	}

	void Window::SetSwapInterval(int interval) noexcept
	{
		p_Impl->SetSwapInterval(interval);
	}

	glm::ivec2 Window::GetWindowDimensions() const noexcept
	{
		return p_Impl->GetWindowDimensions();
	}
	
	glm::ivec2 Window::GetWindowFrameBufferSize() const noexcept
	{
		return p_Impl->GetWindowFrameBufferSize();
	}
	void Window::SetWindowDimensions(const glm::ivec2& dimensions) noexcept
	{
		p_Impl->SetWindowDimensions(dimensions);
	}

}