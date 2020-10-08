#pragma once
#ifndef AUDIOCLIP_HPP
#define AUDIOCLIP_HPP
#include <string>
#include <AL/al.h>
#include <AL/alc.h>
namespace Mona {
	class AudioClip {
	public:
		AudioClip(const std::string& audioFilePath);
		ALuint GetBuffer() const { return m_alBuffer; }
		~AudioClip();
	private:
		ALuint m_alBuffer;
	};
}
#endif