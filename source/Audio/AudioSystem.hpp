#pragma once
#ifndef AUDIOSYSTEM_HPP
#define AUDIOSYSTEM_HPP
#include <AL/al.h>
#include <AL/alc.h>
#include <memory>
#include "AudioClip.hpp"
namespace Mona {
	class AudioSystem {
	public:
		void StartUp() noexcept;
		void ShutDown() noexcept;
	private:
		ALCcontext* m_audioContext;
		ALCdevice* m_audioDevice;
		AudioClip* m_audioClip = nullptr;
		ALuint m_source;
	};
}
#endif