#include "Log.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include "Common.h"
namespace Mona {
	std::shared_ptr<spdlog::logger> Log::m_loggerImplementation;
	void Log::StartUp() noexcept
	{
		ASSERT_MESSAGE(m_loggerImplementation == nullptr, "Logger already Started Up!!");

		spdlog::set_pattern("%^[%T] %n: %v%$");
		m_loggerImplementation = spdlog::stdout_color_mt("MONA");
	}
}

