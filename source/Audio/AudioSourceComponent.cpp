#include "AudioSourceComponent.hpp"
#include <algorithm>
#include <AL/al.h>
#include "../Core/Log.hpp"
#include "AudioMacros.hpp"
namespace Mona {

	AudioSourceComponent::AudioSourceComponent(std::shared_ptr<AudioClip> audioClip /*= nullptr*/,
		float volume /*= 1.0f*/,
		float pitch /*= 1.0f*/,
		bool isLooping /*= false*/,
		float radius /*= 1000.0f*/,
		AudioSourcePriority priority /*= AudioSourcePriority::SoundPriorityMedium*/,
		SourceType sourceType /*= SourceType::Source2D*/) : 
		AudioSource(audioClip, volume,pitch,radius, priority, sourceType),
		m_isLooping(isLooping),
		m_sourceState(AudioSourceState::Stopped)
	{

	}

	void AudioSourceComponent::SetAudioClip(std::shared_ptr<AudioClip> audioClip) noexcept
	{
		if (m_openALsource) {
			const OpenALSource &alSource = m_openALsource.value();
			ALCALL(alSourceStop(alSource.m_sourceID));
			ALCALL(alSourcei(alSource.m_sourceID, AL_BUFFER, audioClip? audioClip->GetBufferID(): 0));
		}
		m_timeLeft = audioClip ? audioClip->GetTotalTime() : 0.0f;
		m_sourceState = AudioSourceState::Stopped;
		m_audioClip = audioClip;
	}



	void AudioSourceComponent::Play() noexcept
	{
		if (!m_audioClip) {
			MONA_LOG_INFO("AudioSourceComponent warning: Trying to play source with no audioClip.");
			return;
		}
		if (m_sourceState == AudioSourceState::Playing) {
			m_timeLeft = m_audioClip->GetTotalTime();
		}
		else {
			m_sourceState = AudioSourceState::Playing;
		}
		
		if (m_openALsource) {
			const OpenALSource& alSource = m_openALsource.value();
			ALCALL(alSourcePlay(alSource.m_sourceID));
		}
		
	}

	void AudioSourceComponent::Stop() noexcept {
		if (!m_audioClip) {
			MONA_LOG_INFO("AudioSourceComponent warning: Trying to stop source with no audioClip.");
			return;
		}

		if (m_sourceState == AudioSourceState::Stopped) return;
		if (m_openALsource) {
			const OpenALSource &alSource = m_openALsource.value();
			ALCALL(alSourceStop(alSource.m_sourceID));
		}
		m_timeLeft = m_audioClip->GetTotalTime();
		m_sourceState = AudioSourceState::Stopped;
	}

	void AudioSourceComponent::Pause() noexcept {
		if (!m_audioClip) {
			MONA_LOG_INFO("AudioSourceComponent warning: Trying to pause source with no audioClip.");
			return;
		}
		if (m_sourceState == AudioSourceState::Paused || m_sourceState == AudioSourceState::Stopped) return;
		if (m_openALsource) {
			const OpenALSource &alSource = m_openALsource.value();
			ALCALL(alSourcePause(alSource.m_sourceID));
		}
		m_sourceState = AudioSourceState::Paused;
	}

	void AudioSourceComponent::SetSourceType(SourceType type) noexcept {
		m_sourceType = type;
		if (m_openALsource)
		{
			const OpenALSource& alSource = m_openALsource.value();
			if (m_sourceType == SourceType::Source2D) {
				ALCALL(alSourcei(alSource.m_sourceID, AL_SOURCE_RELATIVE, AL_TRUE));
				ALCALL(alSource3f(alSource.m_sourceID, AL_POSITION, 0.0f, 0.0f, 0.0f));
			}
			else {
				ALCALL(alSourcei(alSource.m_sourceID, AL_SOURCE_RELATIVE, AL_FALSE));
			}
		}

	}
	void AudioSourceComponent::SetVolume(float volume) noexcept {
		m_volume = std::clamp(volume, 0.0f, 1.0f);
		if (m_openALsource) {
			const OpenALSource& alSource = m_openALsource.value();
			ALCALL(alSourcef(alSource.m_sourceID, AL_GAIN, m_volume));
		}
	}
	void AudioSourceComponent::SetPitch(float pitch) noexcept {
		m_pitch = std::max(0.0f, pitch);
		if (m_openALsource) {
			const OpenALSource& alSource = m_openALsource.value();
			ALCALL(alSourcef(alSource.m_sourceID, AL_PITCH, m_pitch));
		}
	}
	void AudioSourceComponent::SetRadius(float radius) noexcept {
		m_radius = std::max(0.0f, radius);
		if (m_openALsource) {
			const OpenALSource& alSource = m_openALsource.value();
			ALCALL(alSourcef(alSource.m_sourceID, AL_MAX_DISTANCE, m_radius));
			ALCALL(alSourcef(alSource.m_sourceID, AL_REFERENCE_DISTANCE, m_radius * 0.2f));
		}
	}
	void AudioSourceComponent::SetIsLooping(bool looping) noexcept {
		m_isLooping = looping;
		if (m_openALsource) {
			const OpenALSource& alSource = m_openALsource.value();
			ALCALL(alSourcei(alSource.m_sourceID, AL_LOOPING, m_isLooping ? AL_TRUE : AL_FALSE));
		}
	}
}	