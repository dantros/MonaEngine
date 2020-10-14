#pragma once
#ifndef AUDIOCLIP_HPP
#define AUDIOCLIP_HPP
#include <string>
#include <AL/al.h>
#include <AL/alc.h>
namespace Mona {
	class AudioClip {
	public:
		friend class AudioClipManager;
		AudioClip(const std::string& audioFilePath);
		AudioClip(const AudioClip&) = delete;
		AudioClip& operator=(const AudioClip&) = delete;
		ALuint GetBufferID() const { return m_alBufferID; }
		float GetTotalTime() const { return m_totalTime; }
		uint8_t GetChannels() const { return m_channels; }
		uint32_t GetSampleRate() const { return m_sampleRate; }
		~AudioClip();
	private:
		void DeleteOpenALBuffer();
		
		uint32_t m_sampleRate;
		float m_totalTime;
		ALuint m_alBufferID;
		uint8_t m_channels;
	};
}
#endif