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

namespace Mona {
	struct InnerComponentHandle;
	class AudioSystem {
	public:
		void StartUp(unsigned int channels = 32) noexcept;
		void ShutDown() noexcept;
		void ClearSources() noexcept;
		void Update(const InnerComponentHandle &audioListenerTransformHandle,
			float timeStep,
			const TransformComponent::managerType& transformDataManager) noexcept;
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
		
	private:
		void UpdateListener(const glm::vec3& position, const glm::vec3& frontVector, const glm::vec3& upVector);
		void RemoveCompletedFreeAudioSources();
		void UpdateFreeAudioSourcesTimers(float timeStep);
		std::vector<FreeAudioSource>::iterator PartitionAndRemoveOpenALSourceByDistance(const glm::vec3& listenerPosition);
		void AssignOpenALSource(std::vector<FreeAudioSource>::iterator begin, std::vector<FreeAudioSource>::iterator end);
		void RemoveOpenALSource(std::vector<FreeAudioSource>::iterator begin, std::vector<FreeAudioSource>::iterator end);
		void SortFreeAudioSourcesByPriority(std::vector<FreeAudioSource>::iterator end);
		ALCcontext* m_audioContext;
		ALCdevice* m_audioDevice;

		struct OpenALSourceArrayEntry {
			ALuint m_sourceID;
			uint32_t m_nextFreeIndex;
			OpenALSourceArrayEntry(ALuint source, uint32_t nextFreeIndex) :
				m_sourceID(source), m_nextFreeIndex(nextFreeIndex) {}
		};
		AudioSource::OpenALSource GetNextFreeSource();
		void FreeOpenALSource(uint32_t index);
		std::vector<OpenALSourceArrayEntry> m_openALSources;
		uint32_t m_firstFreeOpenALSourceIndex;
		uint32_t m_channels;
		std::vector<FreeAudioSource> m_freeAudioSources;
		//AudioClip* m_audioClip = nullptr;
		//ALuint m_source;
	};
}
#endif