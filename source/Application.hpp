#pragma once
#ifndef APPLICATION_HPP
#define APPLICATION_HPP
#include <memory>
namespace Mona {
	class Window;
	class Input;
	class Engine;
	class Application {
	public:
		friend class Engine;
		virtual void UserStartUp() noexcept = 0;
		virtual void UserShutDown() noexcept = 0;
		virtual void UserUpdate(float timestep)  noexcept = 0;
		virtual ~Application() = default;
	protected:
		std::shared_ptr<Window> m_window;
		std::shared_ptr<Input> m_input;
	private:
		void StartUp(std::shared_ptr<Window> window, std::shared_ptr<Input> input) noexcept;

	};
}
#endif


