#pragma once
#ifndef COMMON_HPP
#define COMMON_HPP
#include <cassert>
#include <stdio.h>
#include <cstdint>
#define NOMINMAX
#ifdef NDEBUG
	#define ASSERT(expr) (void(0))
#else
	#if WIN32
		#define ASSERT(expr) {if (!(expr))  {__debugbreak(); }}
	#else
		#define ASSERT(expr) assert(expr)
	#endif
#endif

#ifdef NDEBUG
	#define ASSERT_MESSAGE(expr, message) (void(0))
#else
	#if WIN32 
		#define ASSERT_MESSAGE(expr, message) \
			{if (!(expr)){ \
			fprintf(stderr, "[%s] ", message); \
			fprintf(stderr, "[Expresion: %s] (%s:%d)", #expr, __FILE__, __LINE__); \
			fprintf(stderr, "\n"); \
			__debugbreak(); }}
			
	#else
		#define ASSERT_MESSAGE(expr, message) \
			{if (!(expr)){ \
			fprintf(stderr, "[%s] ", message); \
			fprintf(stderr, "[Expresion: %s] (%s:%d)", #expr, __FILE__, __LINE__); \
			fprintf(stderr, "\n"); \
			assert(expr);}}
	#endif
#endif


#endif