#pragma once
#ifndef AUDIOSYSTEM_HPP
#define AUDIOSYSTEM_HPP
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <AL/al.h>
#include <AL/alc.h>
#include "../World/ComponentTypes.hpp"
#include "../World/TransformComponent.hpp"
#include "AudioClip.hpp"
#include "AudioSource.hpp"
#include "AudioSourceComponent.hpp"
namespace Mona {
	struct InnerComponentHandle;
	class AudioSystem {
	public:
		void StartUp(unsigned int channels = 32) noexcept;
		void ShutDown() noexcept;
		void Update(const InnerComponentHandle &audioListenerTransformHandle,
			float timeStep,
			const TransformComponent::managerType& transformDataManager,
			AudioSourceComponent::managerType& audioSourceDataManager) noexcept;
		float GetMasterVolume() const noexcept;
		void SetMasterVolume(float volume) noexcept;
		void PlayAudioClip3D(std::shared_ptr<AudioClip> audioClip,
			const glm::vec3& position,
			float volume,
			float pitch,
			float radius,
			AudioSourcePriority priority
			);
		void PlayAudioClip2D(std::shared_ptr<AudioClip> audioClip,
			float volume,
			float pitch,
			AudioSourcePriority priority);

		void RemoveOpenALSource(uint32_t index) noexcept;
		void ClearSources() noexcept;
	private:
		void UpdateListener(const glm::vec3& position, const glm::vec3& frontVector, const glm::vec3& upVector);
		void RemoveCompletedFreeAudioSources();
		void UpdateFreeAudioSourcesTimers(float timeStep);
		void UpdateAudioSourceComponentsTimers(float timeStep, AudioSourceComponent::managerType& audioDataManager);
		std::vector<FreeAudioSource>::iterator PartitionAndRemoveOpenALSourceFromFreeAudioSources(const glm::vec3& listenerPosition);
		uint32_t PartitionAndRemoveOpenALSourceFromAudioSourceComponents(AudioSourceComponent::managerType& audioDataManager,
			const TransformComponent::managerType& transformDataManager,
			const glm::vec3& listenerPosition);
		void AssignOpenALSourceToFreeAudioSources(std::vector<FreeAudioSource>::iterator begin,
			std::vector<FreeAudioSource>::iterator end);
		void AssignOpenALSourceToAudioSourceComponents(AudioSourceComponent::managerType& audioDataManager,
			const TransformComponent::managerType& transformDataManager,
			uint32_t firstIndex,
			uint32_t lastIndex);
		void RemoveOpenALSourceFromFreeAudioSources(std::vector<FreeAudioSource>::iterator begin,
			std::vector<FreeAudioSource>::iterator end);
		void RemoveOpenALSourceFromAudioSourceComponents(AudioSourceComponent::managerType& audioDataManager,
			uint32_t firstIndex,
			uint32_t lastIndex);
		void SortFreeAudioSourcesByPriority(std::vector<FreeAudioSource>::iterator end,
			uint32_t (&outCount)[static_cast<unsigned int>(AudioSourcePriority::PriorityCount)]);
		void SortAudioSourceComponentsByPriority(AudioSourceComponent::managerType& audioDataManager,
			uint32_t lastIndex,
			uint32_t (&outCount)[static_cast<unsigned int>(AudioSourcePriority::PriorityCount)]);
		

	
		void FreeOpenALSource(uint32_t index);
		struct OpenALSourceArrayEntry {
			ALuint m_sourceID;
			uint32_t m_nextFreeIndex;
			OpenALSourceArrayEntry(ALuint source, uint32_t nextFreeIndex) :
				m_sourceID(source), m_nextFreeIndex(nextFreeIndex) {}
		};
		AudioSource::OpenALSource GetNextFreeSource();


		ALCcontext* m_audioContext;
		ALCdevice* m_audioDevice;
		std::vector<OpenALSourceArrayEntry> m_openALSources;
		uint32_t m_firstFreeOpenALSourceIndex;
		uint32_t m_channels;
		std::vector<FreeAudioSource> m_freeAudioSources;
		float m_masterVolume;
	};
}
#endif