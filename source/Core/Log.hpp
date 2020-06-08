#pragma once
#ifndef LOG_HPP
#define LOG_HPP
#include "Common.hpp"
#include <memory>
#include <spdlog/spdlog.h>

namespace Mona {
	class Log {
	public:
		//TODO(Byron) Maybe move StartUp and ShutDown to private and make caller friend class
		static void StartUp() noexcept;
		inline static std::shared_ptr<spdlog::logger>& GetLogger() noexcept{ return s_loggerImplementation; }

	private:
		Log() {}
		Log(const Log& log) = delete;
		Log& operator=(const Log& l) = delete;
		static std::shared_ptr<spdlog::logger> s_loggerImplementation;
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