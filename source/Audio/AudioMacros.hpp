#pragma once
#ifndef AUDIOMACROS_HPP
#define AUDIOMACROS_HPP
#include "../Core/Log.hpp"
#define CHECKOPENALERROR(x)\
{\
	ALenum error = alGetError();\
	MONA_ASSERT(error == AL_NO_ERROR, "OPENAL Error:{0}, while calling function {1}", error, #x);\
}

#if NDEBUG
#define ALCALL(x) x
#else
#define ALCALL(x)\
x;\
CHECKOPENALERROR(x)

#endif

#endif