#include "Log.hpp"
#include <spdlog/sinks/stdout_color_sinks.h>
#include "Common.hpp"
namespace Mona {
	std::shared_ptr<spdlog::logger> Log::s_loggerImplementation;
	void Log::StartUp() noexcept
	{
		ASSERT_MESSAGE(s_loggerImplementation == nullptr, "Logger already Started Up!!");
		spdlog::set_pattern("%^[%T] %n: %v%$");
		s_loggerImplementation = spdlog::stdout_color_mt("MONA");
	}
}

