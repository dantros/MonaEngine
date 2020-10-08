#include "AudioClip.hpp"
#include <dr_wav.h>
#include <vector>
#include <limits>
#include "../Core/Log.hpp"
#include "AudioMacros.hpp"
namespace Mona {

	AudioClip::AudioClip(const std::string& audioFilePath)
	{
		struct WavData {
			unsigned int channels = 0;
			unsigned int sampleRate = 0;
			drwav_uint64 totalPCMFrameCount = 0;
			std::vector<uint16_t> pcmData;
			drwav_uint64 GetTotalSamples() const { return totalPCMFrameCount * channels; }
		};

		WavData audioData;
		drwav_int16* sampleData = drwav_open_file_and_read_pcm_frames_s16(audioFilePath.c_str(), &audioData.channels,
			&audioData.sampleRate,
			&audioData.totalPCMFrameCount,
			nullptr);
		if (!sampleData) {
			MONA_LOG_ERROR("Audio Clip Error: Failed to load file {0}", audioFilePath);
			drwav_free(sampleData, nullptr);
			m_alBuffer = 0;
		}
		else if (audioData.GetTotalSamples() > drwav_uint64(std::numeric_limits<size_t>::max())) {
			MONA_LOG_ERROR("Audio Clip Error: File {0} is to big to be loaded.", audioFilePath);
			drwav_free(sampleData, nullptr);
			m_alBuffer = 0;
		}
		else {
			audioData.pcmData.resize(size_t(audioData.GetTotalSamples()));
			std::memcpy(audioData.pcmData.data(), sampleData, audioData.pcmData.size() * 2);
			drwav_free(sampleData, nullptr);
			ALCALL(alGenBuffers(1, &m_alBuffer));
			ALCALL(alBufferData(m_alBuffer, audioData.channels > 1 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16, audioData.pcmData.data(), audioData.pcmData.size() * 2, audioData.sampleRate));
		}


	}
	AudioClip::~AudioClip() {
		ALCALL(alDeleteBuffers(1, &m_alBuffer));
	}

}