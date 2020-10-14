#include "AudioSource.hpp"
#include "AudioMacros.hpp"
namespace Mona {
	
	/*
	void AudioSource::PlayAudioClip(const std::shared_ptr<AudioClip>& audioClip) noexcept {
		m_audioClip = audioClip;
		if (audioClip) {
			m_timeLeft = audioClip->GetTotalTime();
			m_state = SourceState::Playing;
		}
		else {
			m_state = SourceState::Stopped;
			StopCurrentSound();
		}
	}

	void AudioSource::SetVolume(float volume) noexcept {
		m_volume = std::clamp(volume, 0.0f, 1.0f);
		if (m_openALsource) {
			OpenALSource source = m_openALsource.value();
			ALCALL(alSourcef(source.m_sourceID, AL_GAIN, m_volume));
		}
	}

	void AudioSource::SetRadius(float radius) noexcept {
		m_radius = std::max(0.0f, radius);
		if (m_openALsource) {
			OpenALSource source = m_openALsource.value();
			ALCALL(alSourcef(source.m_sourceID, AL_MAX_DISTANCE, m_radius));
		}
	}
	void AudioSource::SetPitch(float pitch) noexcept {
		m_pitch = std::max(0.0f, pitch);
		if (m_openALsource) {
			OpenALSource source = m_openALsource.value();
			ALCALL(alSourcef(source.m_sourceID, AL_PITCH, m_pitch));
		}
	}

	void AudioSource::SetIsLooping(bool looping) noexcept {
		m_isLooping = looping;
		if (m_openALsource) {
			OpenALSource source = m_openALsource.value();
			ALCALL(alSourcei(source.m_sourceID, AL_LOOPING, m_isLooping ? AL_TRUE : AL_FALSE));
		}
	}

	void AudioSource::Stop() noexcept {
		if (m_audioClip  && m_state == SourceState::Playing) {
			m_state = SourceState::Stopped;
			StopCurrentSound();
		}	
	
	}

	void AudioSource::StopCurrentSound() {
		if (m_openALsource)
		{
			OpenALSource source = m_openALsource.value();
			ALCALL(alSourceStop(source.m_sourceID));
		}
		
	}
	*/

}