#pragma once
#ifndef AUDIOSOURCECOMPONENT_HPP
#define AUDIOSOURCECOMPONENT_HPP
#include <memory>
#include "../World/GameObjectTypes.hpp"
#include "AudioClip.hpp"
#include "AudioSource.hpp"
#include "../World/ComponentTypes.hpp"


namespace Mona {
	enum class AudioSourceState {
		Stopped,
		Playing,
		Paused
	};
	
	class AudioSourceComponent : public AudioSource {
	public:
		using managerType = ComponentManager<AudioSourceComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "AudioSourceComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::AudioSourceComponent);

		friend class AudioSystem;
		AudioSourceComponent(std::shared_ptr<AudioClip> audioClip = nullptr,
			float volume = 1.0f,
			float pitch = 1.0f,
			bool isLooping = false,
			float radius = 1000.0f,
			AudioSourcePriority priority = AudioSourcePriority::SoundPriorityMedium,
			SourceType sourceType = SourceType::Source2D);

		std::shared_ptr<AudioClip> GetAudioClip() const noexcept { return m_audioClip; }
		void SetAudioClip(std::shared_ptr<AudioClip> audioClip) noexcept;
		void Play() noexcept;
		void Stop() noexcept;
		void Pause() noexcept;
		AudioSourceState GetAudioSourceState() const noexcept { return m_sourceState; }
		AudioSourcePriority GetSourcePriority() const noexcept { return m_priority; }
		void SetSourcePriority(AudioSourcePriority priority) noexcept { m_priority = priority; }
		SourceType GetSourceType() const noexcept { return m_sourceType; }
		void SetSourceType(SourceType type) noexcept { m_sourceType = type; }
		float GetVolume() const noexcept { return m_volume; }
		void SetVolume(float volume) noexcept;
		float GetRadius() const noexcept { return m_radius; }
		void SetRadius(float radius) noexcept;
		float GetPitch() const noexcept { return m_pitch; }
		void SetPitch(float pitch) noexcept;
		bool IsLooping() const noexcept { return m_isLooping; }
		void SetIsLooping(bool looping) noexcept;
		float GetTimeLeft() const noexcept { return m_timeLeft; }


	private:
		void SetTransformHandle(const InnerComponentHandle& handle) { m_transformHandle = handle; }
		InnerComponentHandle m_transformHandle;
		AudioSourceState m_sourceState;
		bool m_isLooping;
	};
}
#endif