#pragma once
#ifndef LOG_H
#define LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
namespace Mona {
	class Log {
	public:
		void Init();


	private:
		static std::shared_ptr<spdlog::logger> m_spdlogger;
	};
}
#endif
