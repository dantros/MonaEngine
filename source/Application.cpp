#include "Application.hpp"

namespace Mona {
	void Application::StartUp(std::shared_ptr<Window> window, std::shared_ptr<Input> input) noexcept
	{
		m_window = window;
		m_input = input;
		UserStartUp();
	}
}