#pragma once
#ifndef LOG_H
#define LOG_H
#include "Common.h"
#include <memory>
#include <utility>
#include <spdlog/spdlog.h>

namespace Mona {
	class Log {
	public:
		static void StartUp() noexcept;
		inline static std::shared_ptr<spdlog::logger>& GetLogger() { return m_loggerImplementation; }

	private:
		Log() = default;
		Log(const Log& log) = delete;
		Log& operator=(const Log& l) = delete;
		static std::shared_ptr<spdlog::logger> m_loggerImplementation;
	};
}

#if NDEBUG
	#define MONA_LOG_INFO(...)					(void(0))
	#define MONA_LOG_ERROR(...)					(void(0))
	#define MONA_ASSERT(expr, ...)				(void(0))
#else
	#define MONA_LOG_INFO(...)					::Mona::Log::GetLogger()->info(__VA_ARGS__)
	#define MONA_LOG_ERROR(...)					::Mona::Log::GetLogger()->error(__VA_ARGS__)
	#if WIN32
		#define MONA_ASSERT(expr, ...)					{if(!(expr)){ \
														MONA_LOG_ERROR(__VA_ARGS__); \
														__debugbreak(); }}
	#else
		#define MONA_ASSERT(expr, ...)					{if(!(expr)){ \
														MONA_LOG_ERROR(__VA_ARGS__); \
														assert(expr); }}
	#endif
												

#endif


#endif