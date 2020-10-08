#include "AudioSystem.hpp"
#include "../Core/Log.hpp"
#include "../Core/RootDirectory.hpp"
#include "AudioMacros.hpp"

namespace Mona {
	void AudioSystem::StartUp() noexcept {
		m_audioDevice = alcOpenDevice(nullptr);
		if (!m_audioDevice) {
			MONA_LOG_ERROR("AudioSystem Error: Failed to open audio device.");
			return;
		}
		m_audioContext = alcCreateContext(m_audioDevice, NULL);
		if (!alcMakeContextCurrent(m_audioContext)) {
			MONA_LOG_ERROR("AudioSystem Error: Failed to make audio context current.");
			return;
		}
		ALCALL(alDistanceModel(AL_LINEAR_DISTANCE_CLAMPED));
		ALCALL(alListener3f(AL_POSITION, 0.0f, 0.0f, 0.0f));
		ALCALL(alListener3f(AL_VELOCITY, 0.0f, 0.0f, 0.0f));
		ALfloat forwardAndUpVectors[] = {
			/*Forward*/ 0.0f, 1.0f, 0.0f,
			/*Up*/		0.0f, 0.0f, 1.0f
		};
		ALCALL(alListenerfv(AL_ORIENTATION, forwardAndUpVectors));
		m_audioClip = new AudioClip(SourcePath("Assets/AudioFiles/footsteps.wav").string());
		ALCALL(alGenSources(1, &m_source));
		ALCALL(alSource3f(m_source, AL_POSITION, 1.0f, 0.0f, 0.0f));
		ALCALL(alSource3f(m_source, AL_VELOCITY, 0.0f, 0.0f, 0.0f));
		ALCALL(alSourcei(m_source, AL_LOOPING, AL_TRUE));
		ALCALL(alSourcef(m_source, AL_PITCH, 1.0f));
		ALCALL(alSourcef(m_source, AL_GAIN, 1.0f));
		ALCALL(alSourcei(m_source, AL_BUFFER, m_audioClip->GetBuffer()));
		ALCALL(alSourcePlay(m_source));

	
	}

	void AudioSystem::ShutDown() noexcept {
		ALCALL(alDeleteSources(1, &m_source));
		delete m_audioClip;
		alcMakeContextCurrent(NULL);
		alcDestroyContext(m_audioContext);
		alcCloseDevice(m_audioDevice);
	}
}