#pragma once
#ifndef AUDIOSOURCE_HPP
#define AUDIOSOURCE_HPP
#include <memory>
#include <optional>
#include <glm/glm.hpp>
#include "AudioClip.hpp"
#include <AL/al.h>
#include  <algorithm>
namespace Mona {
	class AudioClip;
	enum class AudioSourcePriority : uint8_t {
		SoundPriorityVerylow,
		SoundPriorityLow,
		SoundPriorityMedium,
		SoundPriorityHigh,
		SoundPriorityVeryHigh,
		PriorityCount
	};

	enum class SourceType : uint8_t{
		Source2D,
		Source3D
	};

	struct AudioSource {
		struct OpenALSource {
			ALuint m_sourceID;
			uint32_t m_sourceIndex;
			OpenALSource(ALuint source = 0, uint32_t index = 0) : m_sourceID(source), m_sourceIndex(index) {}
		};
		std::optional<OpenALSource> m_openALsource ;
		std::shared_ptr<AudioClip> m_audioClip;
		float m_volume;
		float m_pitch;
		float m_radius;
		float m_timeLeft;
		AudioSourcePriority m_priority;
		SourceType m_sourceType;
		AudioSource(std::shared_ptr<AudioClip> audioClip,
			float volume,
			float pitch,
			float radius,
			AudioSourcePriority priority,
			SourceType sourceType) :
			m_audioClip(audioClip),
			m_openALsource(std::nullopt),
			m_volume(volume),
			m_pitch(pitch),
			m_radius(radius),
			m_priority(priority),
			m_sourceType(sourceType),
			m_timeLeft(0.0f)
		{
			if(m_audioClip)
			{
				m_timeLeft = m_audioClip->GetTotalTime();
			}
		}
	};

	class FreeAudioSource : public AudioSource {
	public:
		friend class  AudioSystem;
		FreeAudioSource(std::shared_ptr<AudioClip> audioClip = nullptr,
			float volume = 1.0f,
			float pitch = 1.0f,
			float radius = 1000.0f,
			AudioSourcePriority priority = AudioSourcePriority::SoundPriorityMedium,
			SourceType sourceType = SourceType::Source2D,
			const glm::vec3 &position = glm::vec3(0.0f,0.0f,0.0f)) :
			AudioSource(audioClip, volume, pitch, radius, priority, sourceType),
			m_position(position)
		{}
	private:
		glm::vec3 m_position;
	};
	/*
	class AudioSource {
		friend class AudioSystem;
	public:
		AudioSource() = default;
		std::shared_ptr<AudioClip> GetAudioClip() const noexcept{ return m_audioClip; }
		void PlayAudioClip(const std::shared_ptr<AudioClip>& audioClip) noexcept;
		AudioSourcePriority GetSourcePriority() const noexcept { return m_priority; }
		void SetSourcePriority(AudioSourcePriority priority) noexcept{ m_priority = priority; }
		SourceType GetSourceType() const noexcept { return m_type; }
		void SetSourceType(SourceType type) noexcept { m_type = type; }
		float GetVolume() const noexcept { return m_volume; }
		void SetVolume(float volume) noexcept;
		float GetRadius() const noexcept { return m_radius; }
		void SetRadius(float radius) noexcept;
		float GetPitch() const noexcept { return m_pitch; }
		void SetPitch(float pitch) noexcept;
		bool IsLooping() const noexcept { return m_isLooping; }
		void SetIsLooping(bool looping) noexcept;
		float GetTimeLeft() const noexcept{ return m_timeLeft; }
	private:
		std::shared_ptr<AudioClip> m_audioClip = nullptr;
		AudioSourcePriority m_priority = AudioSourcePriority::SoundPriorityMedium;
		SourceType m_type = SourceType::Source3D;
		float m_volume = 1.0f;
		float m_radius = 1000.0f;
		float m_pitch = 1.0f;
		float m_timeLeft = 0.0f;
		bool m_isLooping = false;
		//bool m_shouldDetachSource = false;
		struct OpenALSource {
			ALuint m_sourceID;
			uint32_t m_sourceIndex;
		};

		std::optional<OpenALSource> m_openALsource = std::nullopt;
	
	};*/
}
#endif