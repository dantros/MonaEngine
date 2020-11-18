#include "AudioClip.hpp"
#include <dr_wav.h>
#include <vector>
#include <limits>
#include "../Core/Log.hpp"
#include "AudioMacros.hpp"
namespace Mona {

	AudioClip::AudioClip(const std::string& audioFilePath) :
		m_sampleRate(0),
		m_totalTime(0.0f),
		m_channels(0)
	{

		struct WavData {
			unsigned int channels = 0;
			unsigned int sampleRate = 0;
			drwav_uint64 totalPCMFrameCount = 0;
			std::vector<uint16_t> pcmData;
			drwav_uint64 GetTotalSamples() const { return totalPCMFrameCount * channels; }
		};

		/*
		* Primero se cargan los datos del archivo ubicado en audioFilePath
		* usando la libreria drwav (https://github.com/mackron/dr_libs)
		*/
		WavData audioData;
		drwav_int16* sampleData = drwav_open_file_and_read_pcm_frames_s16(audioFilePath.c_str(), &audioData.channels,
			&audioData.sampleRate,
			&audioData.totalPCMFrameCount,
			nullptr);
		if (!sampleData) {
			MONA_LOG_ERROR("Audio Clip Error: Failed to load file {0}", audioFilePath);
			drwav_free(sampleData, nullptr);
			m_alBufferID = 0;
		}
		else if (audioData.GetTotalSamples() > drwav_uint64(std::numeric_limits<size_t>::max())) {
			MONA_LOG_ERROR("Audio Clip Error: File {0} is to big to be loaded.", audioFilePath);
			drwav_free(sampleData, nullptr);
			m_alBufferID = 0;
		}
		else {
			
			//Si la carga usando dr_wav fue exitosa se comienza el transpaso de estos datos a OpenAL.
			audioData.pcmData.resize(size_t(audioData.GetTotalSamples()));
			m_totalTime = (float) audioData.totalPCMFrameCount / (float) audioData.sampleRate;
			m_sampleRate = static_cast<uint32_t>(audioData.sampleRate);
			m_channels = static_cast<uint8_t>(audioData.channels);

			//Primero se copian todos los datos a un vector de uint16_t, para luego liberar los datos recien copiados.
			std::memcpy(audioData.pcmData.data(), sampleData, audioData.pcmData.size() * 2);
			drwav_free(sampleData, nullptr);

			//Se pasa este vector de uint16_t a OpenAL
			ALCALL(alGenBuffers(1, &m_alBufferID));
			MONA_ASSERT(m_alBufferID, "AudioClip Error: OpenAL wasn't able to load audioclip from {0}", audioFilePath);
			ALCALL(alBufferData(m_alBufferID, audioData.channels > 1 ? AL_FORMAT_STEREO16 : AL_FORMAT_MONO16, audioData.pcmData.data(), audioData.pcmData.size() * 2, audioData.sampleRate));
		}


	}

	void AudioClip::DeleteOpenALBuffer() {

		ALCALL(alDeleteBuffers(1, &m_alBufferID));
		m_alBufferID = 0;
	}
	AudioClip::~AudioClip() {
		if (m_alBufferID)
		{
			ALCALL(alDeleteBuffers(1, &m_alBufferID));
		}
			
	}


}